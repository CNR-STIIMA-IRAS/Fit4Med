#include <ecrt.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static constexpr unsigned MASTER_INDEX = 1;
static constexpr uint32_t DRIVE_VENDOR = 0x000001dd;
static constexpr uint32_t DRIVE_PRODUCT = 0x10305070;
static constexpr uint32_t ATI_VENDOR = 0x00000732;
static constexpr uint32_t ATI_PRODUCT = 0x26483053;

static constexpr uint64_t DEFAULT_CYCLE_NS = 1000000; // 1 ms
static constexpr unsigned RESET_RETRY_CYCLES = 1000;
static constexpr unsigned DEFAULT_RESET_PULSE_HIGH_CYCLES = 20;
static constexpr unsigned DEFAULT_RESET_PULSE_TRAILING_LOW_CYCLES = 20;
static constexpr unsigned DEFAULT_SUCCESS_STABLE_CYCLES = 200;
static constexpr unsigned EXPECTED_SLAVES = 4;
static constexpr unsigned EXIT_SHUTDOWN_CYCLES = 20;
static constexpr unsigned EXIT_DISABLE_VOLTAGE_CYCLES = 20;

static volatile std::sig_atomic_t g_run = 1;

static void signal_handler(int) { g_run = 0; }

static uint64_t timespec_to_ns(const timespec &ts) {
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

static timespec ns_to_timespec(uint64_t ns) {
  timespec ts{};
  ts.tv_sec = ns / 1000000000ULL;
  ts.tv_nsec = ns % 1000000000ULL;
  return ts;
}

static void set_realtime() {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall");
  }

  sched_param param{};
  param.sched_priority = 80;

  if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
    perror("sched_setscheduler");
    std::cerr << "Warning: not running SCHED_FIFO. Try running with sudo.\n";
  }
}

struct DriveOffsets {
  unsigned controlword;
  unsigned target_position;
  unsigned target_velocity;
  unsigned mode_of_operation;
  unsigned velocity_offset;

  unsigned statusword;
  unsigned actual_position;
  unsigned actual_velocity;
  unsigned actual_torque;
  unsigned mode_display;
};

struct AtiOffsets {
  unsigned control_1;
  unsigned control_2;

  unsigned fx;
  unsigned fy;
  unsigned fz;
  unsigned tx;
  unsigned ty;
  unsigned tz;
  unsigned status_code;
  unsigned sample_counter;
};

static std::array<DriveOffsets, 3> drive_offsets;
static AtiOffsets ati_offsets;

static ec_master_t *master = nullptr;
static ec_domain_t *domain = nullptr;
static uint8_t *domain_pd = nullptr;

static ec_pdo_entry_info_t drive_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32}, {0x60ff, 0x00, 32},
    {0x6060, 0x00, 8},  {0x60b1, 0x00, 32}, {0x6041, 0x00, 16},
    {0x6064, 0x00, 32}, {0x606c, 0x00, 32}, {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
};

static ec_pdo_info_t drive_pdos[] = {
    {0x1601, 5, drive_pdo_entries + 0},
    {0x1a01, 5, drive_pdo_entries + 5},
};

static ec_sync_info_t drive_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, drive_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, drive_pdos + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE}};

static ec_pdo_entry_info_t ati_pdo_entries[] = {
    {0x7010, 0x01, 32}, {0x7010, 0x02, 32}, {0x6000, 0x01, 32},
    {0x6000, 0x02, 32}, {0x6000, 0x03, 32}, {0x6000, 0x04, 32},
    {0x6000, 0x05, 32}, {0x6000, 0x06, 32}, {0x6010, 0x00, 32},
    {0x6020, 0x00, 32},
};

static ec_pdo_info_t ati_pdos[] = {
    {0x1601, 2, ati_pdo_entries + 0},
    {0x1a00, 8, ati_pdo_entries + 2},
};

static ec_sync_info_t ati_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, ati_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, ati_pdos + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE}};

static ec_pdo_entry_reg_t domain_regs[] = {
    // Drive 0
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6040, 0x00,
     &drive_offsets[0].controlword, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x607a, 0x00,
     &drive_offsets[0].target_position, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x60ff, 0x00,
     &drive_offsets[0].target_velocity, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6060, 0x00,
     &drive_offsets[0].mode_of_operation, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x60b1, 0x00,
     &drive_offsets[0].velocity_offset, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6041, 0x00,
     &drive_offsets[0].statusword, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6064, 0x00,
     &drive_offsets[0].actual_position, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x606c, 0x00,
     &drive_offsets[0].actual_velocity, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6077, 0x00,
     &drive_offsets[0].actual_torque, nullptr},
    {0, 0, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6061, 0x00,
     &drive_offsets[0].mode_display, nullptr},

    // Drive 1
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6040, 0x00,
     &drive_offsets[1].controlword, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x607a, 0x00,
     &drive_offsets[1].target_position, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x60ff, 0x00,
     &drive_offsets[1].target_velocity, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6060, 0x00,
     &drive_offsets[1].mode_of_operation, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x60b1, 0x00,
     &drive_offsets[1].velocity_offset, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6041, 0x00,
     &drive_offsets[1].statusword, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6064, 0x00,
     &drive_offsets[1].actual_position, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x606c, 0x00,
     &drive_offsets[1].actual_velocity, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6077, 0x00,
     &drive_offsets[1].actual_torque, nullptr},
    {0, 1, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6061, 0x00,
     &drive_offsets[1].mode_display, nullptr},

    // Drive 2
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6040, 0x00,
     &drive_offsets[2].controlword, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x607a, 0x00,
     &drive_offsets[2].target_position, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x60ff, 0x00,
     &drive_offsets[2].target_velocity, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6060, 0x00,
     &drive_offsets[2].mode_of_operation, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x60b1, 0x00,
     &drive_offsets[2].velocity_offset, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6041, 0x00,
     &drive_offsets[2].statusword, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6064, 0x00,
     &drive_offsets[2].actual_position, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x606c, 0x00,
     &drive_offsets[2].actual_velocity, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6077, 0x00,
     &drive_offsets[2].actual_torque, nullptr},
    {0, 2, DRIVE_VENDOR, DRIVE_PRODUCT, 0x6061, 0x00,
     &drive_offsets[2].mode_display, nullptr},

    // ATI F/T sensor, slave 3
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x7010, 0x01, &ati_offsets.control_1,
     nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x7010, 0x02, &ati_offsets.control_2,
     nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6000, 0x01, &ati_offsets.fx, nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6000, 0x02, &ati_offsets.fy, nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6000, 0x03, &ati_offsets.fz, nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6000, 0x04, &ati_offsets.tx, nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6000, 0x05, &ati_offsets.ty, nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6000, 0x06, &ati_offsets.tz, nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6010, 0x00, &ati_offsets.status_code,
     nullptr},
    {0, 3, ATI_VENDOR, ATI_PRODUCT, 0x6020, 0x00, &ati_offsets.sample_counter,
     nullptr},

    {}};

static void reset_master_or_throw() {
  const int ret = ecrt_master_reset(master);
  if (ret != 0) {
    const int err = ret < 0 ? -ret : ret;
    throw std::runtime_error(std::string("EtherCAT master reset failed: ") +
                             std::strerror(err));
  }
}

static void configure_network(uint64_t cycle_ns, bool enable_dc,
                              uint32_t dc_assign) {
  // Retry on EBUSY: a previous process may still be releasing the master.
  static constexpr unsigned MASTER_REQUEST_RETRIES = 10;
  for (unsigned attempt = 1; attempt <= MASTER_REQUEST_RETRIES; ++attempt) {
    master = ecrt_request_master(MASTER_INDEX);
    if (master) break;
    if (attempt == MASTER_REQUEST_RETRIES) {
      throw std::runtime_error("Failed to request EtherCAT master " +
                               std::to_string(MASTER_INDEX) +
                               " after " + std::to_string(MASTER_REQUEST_RETRIES) +
                               " attempts");
    }
    std::cout << "Master " << MASTER_INDEX << " busy, retrying ("
              << attempt << "/" << MASTER_REQUEST_RETRIES << ")...\n";
    sleep(1);
  }

  std::cout << "Requested EtherCAT master " << MASTER_INDEX << "\n";

  domain = ecrt_master_create_domain(master);
  if (!domain) {
    throw std::runtime_error("Failed to create domain");
  }

  for (unsigned i = 0; i < 3; ++i) {
    ec_slave_config_t *sc =
        ecrt_master_slave_config(master, 0, i, DRIVE_VENDOR, DRIVE_PRODUCT);

    if (!sc) {
      throw std::runtime_error("Failed to get drive slave config " +
                               std::to_string(i));
    }

    if (ecrt_slave_config_pdos(sc, EC_END, drive_syncs)) {
      throw std::runtime_error("Failed to configure PDOs for drive " +
                               std::to_string(i));
    }

    if (enable_dc) {
      ecrt_slave_config_dc(sc, dc_assign, cycle_ns, 0, 0, 0);
      std::cout << "Drive " << i << ": DC enabled, assign=0x" << std::hex
                << dc_assign << std::dec << ", cycle=" << cycle_ns << " ns\n";
    }
  }

  {
    ec_slave_config_t *sc =
        ecrt_master_slave_config(master, 0, 3, ATI_VENDOR, ATI_PRODUCT);

    if (!sc) {
      throw std::runtime_error("Failed to get ATI sensor slave config");
    }

    if (ecrt_slave_config_pdos(sc, EC_END, ati_syncs)) {
      throw std::runtime_error("Failed to configure PDOs for ATI sensor");
    }
  }

  if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
    throw std::runtime_error("PDO entry registration failed");
  }

  std::cout << "Activating master...\n";
  if (ecrt_master_activate(master)) {
    throw std::runtime_error("Failed to activate master");
  }

  domain_pd = ecrt_domain_data(domain);
  if (!domain_pd) {
    throw std::runtime_error("Failed to get domain data pointer");
  }

  std::cout << "Resetting EtherCAT master/slaves at AL level...\n";
  reset_master_or_throw();

  std::cout << "Master activated.\n";
}

enum class Ds402State {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  Unknown,
};

static Ds402State ds402_state_from_status(uint16_t sw) {
  if ((sw & 0x004F) == 0x0000) {
    return Ds402State::NotReadyToSwitchOn;
  }

  if ((sw & 0x004F) == 0x0040) {
    return Ds402State::SwitchOnDisabled;
  }

  if ((sw & 0x006F) == 0x0021) {
    return Ds402State::ReadyToSwitchOn;
  }

  if ((sw & 0x006F) == 0x0023) {
    return Ds402State::SwitchedOn;
  }

  if ((sw & 0x006F) == 0x0027) {
    return Ds402State::OperationEnabled;
  }

  if ((sw & 0x006F) == 0x0007) {
    return Ds402State::QuickStopActive;
  }

  if ((sw & 0x004F) == 0x000F) {
    return Ds402State::FaultReactionActive;
  }

  if ((sw & 0x004F) == 0x0008) {
    return Ds402State::Fault;
  }

  return Ds402State::Unknown;
}

static const char *ds402_state_name(Ds402State state) {
  switch (state) {
  case Ds402State::NotReadyToSwitchOn:
    return "NOT_READY_TO_SWITCH_ON";
  case Ds402State::SwitchOnDisabled:
    return "SWITCH_ON_DISABLED";
  case Ds402State::ReadyToSwitchOn:
    return "READY_TO_SWITCH_ON";
  case Ds402State::SwitchedOn:
    return "SWITCHED_ON";
  case Ds402State::OperationEnabled:
    return "OPERATION_ENABLED";
  case Ds402State::QuickStopActive:
    return "QUICK_STOP_ACTIVE";
  case Ds402State::FaultReactionActive:
    return "FAULT_REACTION_ACTIVE";
  case Ds402State::Fault:
    return "FAULT";
  case Ds402State::Unknown:
    return "UNKNOWN";
  }

  return "UNKNOWN";
}

static bool ds402_is_fault(uint16_t sw) {
  return ds402_state_from_status(sw) == Ds402State::Fault;
}

static void print_states() {
  ec_master_state_t ms{};
  ec_domain_state_t ds{};

  ecrt_master_state(master, &ms);
  ecrt_domain_state(domain, &ds);

  std::cout << "Master: slaves=" << ms.slaves_responding << ", al_states=0x"
            << std::hex << static_cast<unsigned>(ms.al_states)
            << ", link=" << std::dec << static_cast<unsigned>(ms.link_up)
            << " | Domain WC=" << ds.working_counter
            << ", WC state=" << ds.wc_state << "\n";

  for (unsigned i = 0; i < 3; ++i) {
    uint16_t sw = EC_READ_U16(domain_pd + drive_offsets[i].statusword);
    uint16_t cw = EC_READ_U16(domain_pd + drive_offsets[i].controlword);
    const Ds402State state = ds402_state_from_status(sw);

    std::cout << "  Drive " << i << ": CW=0x" << std::hex << std::setw(4)
              << std::setfill('0') << cw << " SW=0x" << std::setw(4) << sw
              << std::dec << std::setfill(' ')
              << " STATE=" << ds402_state_name(state);

    if (sw & 0x0010) {
      std::cout << " VOLTAGE_ENABLED";
    }

    if (sw & 0x0200) {
      std::cout << " REMOTE";
    }

    if (sw & 0x0400) {
      std::cout << " TARGET_REACHED";
    }

    if (sw & 0x0800) {
      std::cout << " INTERNAL_LIMIT";
    }

    std::cout << "\n";
  }

  uint32_t ati_status = EC_READ_U32(domain_pd + ati_offsets.status_code);
  uint32_t ati_counter = EC_READ_U32(domain_pd + ati_offsets.sample_counter);

  std::cout << "  ATI: status=0x" << std::hex << ati_status
            << " counter=" << std::dec << ati_counter << "\n";
}

struct FaultResetState {
  unsigned fault_cycles = 0;
  unsigned reset_phase = 0;
  bool prev_fault = false;
};

static uint16_t ds402_reset_controlword_from_status(uint16_t sw,
                                                    FaultResetState &state,
                                                    unsigned drive_index,
                                                    unsigned pulse_high_cycles,
                                                    unsigned pulse_trailing_low_cycles) {
  const bool fault_active = ds402_is_fault(sw);

  if (!fault_active && state.reset_phase == 0) {
    state.fault_cycles = 0;
    state.prev_fault = false;
    return 0x0000;
  }

  if (fault_active) {
    ++state.fault_cycles;
  }

  // On fault rising edge and then every retry period, emit a short
  // 0x0000 -> 0x0080 -> 0x0000 reset edge.
  if (fault_active && state.reset_phase == 0 &&
      (!state.prev_fault || (state.fault_cycles % RESET_RETRY_CYCLES) == 0)) {
    state.reset_phase = 1;
    std::cout << "Drive " << drive_index << ": pulsing DS402 fault reset\n";
  }

  const uint16_t cw = (state.reset_phase >= 2 &&
                       state.reset_phase <= (pulse_high_cycles + 1))
                          ? 0x0080
                          : 0x0000;

  const unsigned pulse_total_cycles =
      1 + pulse_high_cycles + pulse_trailing_low_cycles;

  if (state.reset_phase > 0) {
    ++state.reset_phase;
    if (state.reset_phase > pulse_total_cycles) {
      state.fault_cycles = 0;
      state.reset_phase = 0;
      state.prev_fault = fault_active;
    }
  }

  if (fault_active) {
    state.prev_fault = true;
  }

  return cw;
}

static uint16_t ds402_enable_controlword_from_status(uint16_t sw) {
  // Basic CiA-402 state machine.
  // This is only used when --enable is passed.
  switch (ds402_state_from_status(sw)) {
  case Ds402State::SwitchOnDisabled:
    return 0x0006; // shutdown
  case Ds402State::ReadyToSwitchOn:
    return 0x0007; // switch on
  case Ds402State::SwitchedOn:
    return 0x000F; // enable operation
  case Ds402State::QuickStopActive:
    return 0x000F; // quick stop active -> re-enable operation
  case Ds402State::OperationEnabled:
    return 0x000F; // stay enabled
  case Ds402State::NotReadyToSwitchOn:
  case Ds402State::FaultReactionActive:
  case Ds402State::Fault:
  case Ds402State::Unknown:
    return 0x0000;
  }

  return 0x0000;
}

static bool ds402_is_healthy_for_exit(uint16_t sw, bool enable_after_reset) {
  const Ds402State state = ds402_state_from_status(sw);

  if (enable_after_reset) {
    return state == Ds402State::OperationEnabled;
  }

  return state != Ds402State::Fault &&
         state != Ds402State::FaultReactionActive &&
         state != Ds402State::Unknown &&
         state != Ds402State::NotReadyToSwitchOn;
}

static void transmit_disable_sequence_before_exit(uint64_t cycle_ns,
                                                  bool enable_dc) {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t next_ns = timespec_to_ns(now);

  auto run_phase = [&](uint16_t controlword, unsigned cycles) {
    for (unsigned step = 0; step < cycles; ++step) {
      next_ns += cycle_ns;
      timespec next_ts = ns_to_timespec(next_ns);
      const int sleep_ret =
          clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_ts, nullptr);
      if (sleep_ret != 0 && sleep_ret != EINTR) {
        throw std::runtime_error(std::string("clock_nanosleep failed: ") +
                                 std::strerror(sleep_ret));
      }

      ecrt_master_receive(master);
      ecrt_domain_process(domain);

      if (enable_dc) {
        ecrt_master_application_time(master, next_ns);
      }

      EC_WRITE_U32(domain_pd + ati_offsets.control_1, 0);
      EC_WRITE_U32(domain_pd + ati_offsets.control_2, 0);

      for (unsigned i = 0; i < 3; ++i) {
        const int32_t actual_position =
            EC_READ_S32(domain_pd + drive_offsets[i].actual_position);

        EC_WRITE_U16(domain_pd + drive_offsets[i].controlword, controlword);
        EC_WRITE_S32(domain_pd + drive_offsets[i].target_position,
                     actual_position);
        EC_WRITE_S32(domain_pd + drive_offsets[i].target_velocity, 0);
        EC_WRITE_S32(domain_pd + drive_offsets[i].velocity_offset, 0);
      }

      if (enable_dc) {
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
      }

      ecrt_domain_queue(domain);
      ecrt_master_send(master);
    }
  };

  std::cout << "Disabling drives before exit (shutdown -> disable voltage).\n";
  run_phase(0x0006, EXIT_SHUTDOWN_CYCLES);
  run_phase(0x0000, EXIT_DISABLE_VOLTAGE_CYCLES);
}

static void cyclic_loop(uint64_t cycle_ns, bool enable_after_reset,
                        bool enable_dc, int8_t mode_of_operation,
                        unsigned success_stable_cycles,
                        unsigned pulse_high_cycles,
                        unsigned pulse_trailing_low_cycles) {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);

  uint64_t next_ns = timespec_to_ns(now);
  unsigned cycle = 0;
  unsigned healthy_stable_cycles = 0;
  std::array<FaultResetState, 3> reset_states{};

  while (g_run != 0) {
    next_ns += cycle_ns;
    timespec next_ts = ns_to_timespec(next_ns);
    const int sleep_ret =
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_ts, nullptr);
    if (sleep_ret != 0 && sleep_ret != EINTR) {
      throw std::runtime_error(std::string("clock_nanosleep failed: ") +
                               std::strerror(sleep_ret));
    }

    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    if (enable_dc) {
      ecrt_master_application_time(master, next_ns);
    }

    // Keep ATI output SyncManager alive.
    EC_WRITE_U32(domain_pd + ati_offsets.control_1, 0);
    EC_WRITE_U32(domain_pd + ati_offsets.control_2, 0);

    bool all_drives_healthy = true;
    for (unsigned i = 0; i < 3; ++i) {
      uint16_t sw = EC_READ_U16(domain_pd + drive_offsets[i].statusword);
      int32_t actual_position =
          EC_READ_S32(domain_pd + drive_offsets[i].actual_position);
        uint16_t cw = ds402_reset_controlword_from_status(
          sw, reset_states[i], i, pulse_high_cycles, pulse_trailing_low_cycles);

      if (ds402_is_fault(sw) || reset_states[i].reset_phase != 0) {
        // Keep driving the reset waveform until the full low-high-low
        // sequence has been transmitted.
      } else if (enable_after_reset) {
        cw = ds402_enable_controlword_from_status(sw);
      } else {
        cw = 0x0000;
      }

      EC_WRITE_U16(domain_pd + drive_offsets[i].controlword, cw);

      // Safe neutral PDO values.
      EC_WRITE_S32(domain_pd + drive_offsets[i].target_position,
                   actual_position);
      EC_WRITE_S32(domain_pd + drive_offsets[i].target_velocity, 0);
      EC_WRITE_S32(domain_pd + drive_offsets[i].velocity_offset, 0);

      if (enable_after_reset) {
        EC_WRITE_S8(domain_pd + drive_offsets[i].mode_of_operation,
                    mode_of_operation);
      }

      const bool drive_healthy =
          (reset_states[i].reset_phase == 0) &&
          ds402_is_healthy_for_exit(sw, enable_after_reset);
      all_drives_healthy = all_drives_healthy && drive_healthy;
    }

    ec_master_state_t ms{};
    ecrt_master_state(master, &ms);
    const bool network_ready =
        (ms.slaves_responding >= EXPECTED_SLAVES) && (ms.link_up != 0);

    if (all_drives_healthy && network_ready) {
      ++healthy_stable_cycles;
    } else {
      healthy_stable_cycles = 0;
    }

    if (healthy_stable_cycles >= success_stable_cycles) {
      std::cout << "All drives healthy and stable for "
                << success_stable_cycles << " cycles. Exiting.\n";
      break;
    }

    if (enable_dc) {
      ecrt_master_sync_reference_clock(master);
      ecrt_master_sync_slave_clocks(master);
    }

    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    if ((cycle % 1000) == 0) {
      print_states();
    }

    ++cycle;
  }

  if (enable_after_reset) {
    try {
      transmit_disable_sequence_before_exit(cycle_ns, enable_dc);
    } catch (const std::exception &e) {
      std::cerr << "Warning: failed to transmit disable sequence: "
                << e.what() << "\n";
    }
  }
}

int main(int argc, char **argv) {
  uint64_t cycle_ns = DEFAULT_CYCLE_NS;
  bool enable_after_reset = false;
  bool enable_dc = false;
  uint32_t dc_assign = 0x0300;
  int8_t mode_of_operation = 8; // CSP by default if --enable is used.
  unsigned success_stable_cycles = DEFAULT_SUCCESS_STABLE_CYCLES;
  unsigned pulse_high_cycles = DEFAULT_RESET_PULSE_HIGH_CYCLES;
  unsigned pulse_trailing_low_cycles = DEFAULT_RESET_PULSE_TRAILING_LOW_CYCLES;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--enable") {
      enable_after_reset = true;
    } else if (arg == "--cycle-ns" && i + 1 < argc) {
      cycle_ns = std::stoull(argv[++i]);
      if (cycle_ns == 0) {
        std::cerr << "Invalid --cycle-ns value: must be > 0\n";
        return 1;
      }
    } else if (arg == "--dc") {
      enable_dc = true;
    } else if (arg == "--dc-assign" && i + 1 < argc) {
      dc_assign = static_cast<uint32_t>(std::stoul(argv[++i], nullptr, 0));
    } else if (arg == "--mode" && i + 1 < argc) {
      const int mode = std::stoi(argv[++i]);
      if (mode < std::numeric_limits<int8_t>::min() ||
          mode > std::numeric_limits<int8_t>::max()) {
        std::cerr << "Invalid --mode value: must fit int8 range [-128, 127]\n";
        return 1;
      }
      mode_of_operation = static_cast<int8_t>(mode);
    } else if (arg == "--success-cycles" && i + 1 < argc) {
      const unsigned long parsed = std::stoul(argv[++i]);
      if (parsed == 0 || parsed > std::numeric_limits<unsigned>::max()) {
        std::cerr << "Invalid --success-cycles value: must be in [1, "
                  << std::numeric_limits<unsigned>::max() << "]\n";
        return 1;
      }
      success_stable_cycles = static_cast<unsigned>(parsed);
    } else if (arg == "--pulse-high-cycles" && i + 1 < argc) {
      const unsigned long parsed = std::stoul(argv[++i]);
      if (parsed == 0 || parsed > std::numeric_limits<unsigned>::max()) {
        std::cerr << "Invalid --pulse-high-cycles value: must be in [1, "
                  << std::numeric_limits<unsigned>::max() << "]\n";
        return 1;
      }
      pulse_high_cycles = static_cast<unsigned>(parsed);
    } else if (arg == "--pulse-low-cycles" && i + 1 < argc) {
      const unsigned long parsed = std::stoul(argv[++i]);
      if (parsed > std::numeric_limits<unsigned>::max()) {
        std::cerr << "Invalid --pulse-low-cycles value: must be in [0, "
                  << std::numeric_limits<unsigned>::max() << "]\n";
        return 1;
      }
      pulse_trailing_low_cycles = static_cast<unsigned>(parsed);
    } else if (arg == "--cycle-ns" || arg == "--dc-assign" ||
               arg == "--mode" || arg == "--success-cycles" ||
               arg == "--pulse-high-cycles" || arg == "--pulse-low-cycles") {
      std::cerr << "Missing value for argument: " << arg << "\n";
      return 1;
    } else if (arg == "--help") {
      std::cout
          << "Usage: " << argv[0] << " [options]\n\n"
          << "Options:\n"
          << "  --cycle-ns N       Cycle time in ns, default 1000000\n"
          << "  --success-cycles N Healthy cycles before auto-exit, default "
          << DEFAULT_SUCCESS_STABLE_CYCLES << "\n"
           << "  --pulse-high-cycles N Reset pulse high duration, default "
           << DEFAULT_RESET_PULSE_HIGH_CYCLES << "\n"
           << "  --pulse-low-cycles N  Reset pulse trailing low duration, "
             "default "
           << DEFAULT_RESET_PULSE_TRAILING_LOW_CYCLES << "\n"
          << "  --dc               Configure DC on drives 0..2\n"
          << "  --dc-assign HEX    DC assignActivate value, default 0x0300\n"
          << "  --enable           After reset, run DS402 0x0006 -> 0x0007 -> "
             "0x000F\n"
          << "  --mode N           DS402 mode when --enable is used, default 8 "
             "CSP\n"
          << "                     Supported here: 8=CSP, 9=CSV\n"
          << "\n"
          << "Default behavior: reset faults only, do not enable motors.\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return 1;
    }
  }

  if (enable_after_reset && mode_of_operation != 8 && mode_of_operation != 9) {
    std::cerr << "Unsupported --mode for this PDO map: "
              << static_cast<int>(mode_of_operation)
              << ". Supported modes are 8 (CSP) and 9 (CSV). Mode 10 (CST) "
                 "requires torque command PDOs such as 0x6071/0x60B2.\n";
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    set_realtime();

    std::cout << "Cycle time: " << cycle_ns << " ns\n";
    std::cout << "Success stable cycles: " << success_stable_cycles << "\n";
    std::cout << "Reset pulse cycles (high/low): " << pulse_high_cycles << "/"
          << pulse_trailing_low_cycles << "\n";
    std::cout << "Enable after reset: " << (enable_after_reset ? "yes" : "no")
              << "\n";
    std::cout << "DC configure: " << (enable_dc ? "yes" : "no") << "\n";
    if (enable_after_reset) {
      std::cout << "Mode of operation: " << static_cast<int>(mode_of_operation)
                << "\n";
    }

    configure_network(cycle_ns, enable_dc, dc_assign);
    cyclic_loop(cycle_ns, enable_after_reset, enable_dc, mode_of_operation,
            success_stable_cycles, pulse_high_cycles,
            pulse_trailing_low_cycles);

    std::cout << "Stopping.\n";
    ecrt_release_master(master);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Fatal: " << e.what() << "\n";

    if (master) {
      ecrt_release_master(master);
    }

    return 1;
  }
}
