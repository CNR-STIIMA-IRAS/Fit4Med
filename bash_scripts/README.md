# Aggiornamento, backup e ripristino del software del robot

Questi tre script servono a portare il software del controllore sul robot, a salvarne una copia e a tornare a una copia precedente. Esistono per Linux (in questa cartella) e per Windows (in `ps_scripts`, vedi [Da Windows](#da-windows)):

| Script | Cosa fa |
| --- | --- |
| `fit4med_to_robot_sync.sh` | Copia sul robot il software che hai sul tuo PC. Prima propone un backup. |
| `fit4med_backup.sh` | Salva una copia del software del robot. |
| `fit4med_restore.sh` | Elenca le copie salvate e ne ripristina una. |

Il robot (`192.168.1.1`) non è collegato a internet. Il software aggiornato si scarica sul tuo PC, che è collegato a internet, e da lì si copia sul robot con `fit4med_to_robot_sync.sh`.

## Dove si trovano le cose

| Cosa | Dove |
| --- | --- |
| Software sul tuo PC | la cartella `src` del workspace che contiene la tua copia del repository `Fit4Med` (vedi sotto) |
| Software sul robot | `/home/fit4med/fit4med_ws/src/` |
| Backup sul robot | `/home/fit4med/bkp/AAAAMMGG/HHMM/`, per esempio `/home/fit4med/bkp/20260925/1430/` |

**Il tuo workspace può stare dove vuoi.** Lo script usa la cartella `src` che contiene la copia del repository da cui lo lanci. Per esempio, se il repository è in `/home/mario/lavoro/fit4med_ws/src/Fit4Med`, copia `/home/mario/lavoro/fit4med_ws/src/`. Per usare un'altra cartella: `--local-path <percorso della cartella src>`.

Per sicurezza la cartella deve chiamarsi `src`. Se hai clonato il repository fuori da un workspace (per esempio in `~/codice/Fit4Med`), lo script si ferma: altrimenti copierebbe sul robot tutto il contenuto di `~/codice`.

## Prima di iniziare

- Il tuo PC deve essere collegato alla rete del robot e raggiungere `192.168.1.1`.
- Ti serve l'accesso ssh all'utente `fit4med` del robot. Senza chiave ssh la password viene chiesta, ma una volta sola per esecuzione. Per non doverla scrivere mai, installa la tua chiave una volta:

  ```bash
  ssh-copy-id fit4med@192.168.1.1
  ```

- **Ferma il robot prima di aggiornare o ripristinare il software.** Dal PC Windows: `ps_scripts\fmrr_cleanup.ps1`.
- Gli script si lanciano dalla cartella `bash_scripts` della tua copia del repository, per esempio:

  ```bash
  cd ~/fit4med_ws/src/Fit4Med/bash_scripts
  ```

## Copiare il software sul robot

```bash
./fit4med_to_robot_sync.sh
```

Ogni cartella che hai nel tuo `src` diventa **identica** sul robot: le modifiche fatte direttamente sul robot vengono sovrascritte, e i file che dentro quella cartella esistono solo sul robot vengono cancellati (con conferma).

**Le cartelle del robot che tu non hai non vengono toccate.** Per esempio, se nel tuo `src` c'è solo `Fit4Med` e sul robot ci sono anche `ethercat_controller` e altri pacchetti, viene aggiornata solo `Fit4Med` e gli altri pacchetti restano come sono. Chi ha in locale tutto il workspace aggiorna invece tutto.

Lo script:

1. controlla che il robot risponda e che ssh funzioni;
2. chiede se fare un backup del software attuale del robot (risposta predefinita: sì). Se il backup fallisce, si ferma senza toccare nulla;
3. se sul robot ci sono file che sul tuo PC non esistono, li elenca e chiede se cancellarli (risposta predefinita: no). Confermare autorizza l'intera sync; se rispondi no, il robot resta com'era;
4. se non ci sono cancellazioni ma ci sono file da aggiungere o aggiornare, li elenca e chiede conferma prima di applicarli (risposta predefinita: no). Se rispondi no, il robot resta com'era;
5. copia i file.

Le cartelle `.git` e le cache di Python (`__pycache__`, `*.pyc`) sul robot non vengono né sovrascritte né cancellate.

**Per vedere cosa cambierebbe senza modificare nulla:**

```bash
./fit4med_to_robot_sync.sh --dry-run
```

Le righe `*deleting` sono i file che verrebbero cancellati dal robot, le altre quelli che verrebbero copiati. In questa modalità non si fa nessun backup.

**Opzioni utili:**

| Opzione | Effetto |
| --- | --- |
| `--dry-run` | Mostra cosa cambierebbe, senza modificare nulla. |
| `--folder <cartella>` | Copia solo una cartella, relativa a `~/fit4med_ws/src/`, per esempio `--folder Fit4Med/rehab_gui`. Anche le cancellazioni restano dentro quella cartella. |
| `--backup` | Fa il backup senza chiedere. |
| `--no-backup` | Non fa il backup e non chiede. |
| `--yes` | Applica gli aggiornamenti e cancella i file presenti solo sul robot senza chiedere conferma. |
| `--host <ip>` | Usa un altro indirizzo del robot. |
| `--local-path <cartella>` | Usa un'altra cartella `src` del tuo PC. |
| `--delete-extra-folders` | Cancella dal robot anche le cartelle che nel tuo `src` non esistono (con conferma). Serve solo per eliminare dal robot un intero pacchetto. |

> **Attenzione:** con `--delete-extra-folders` tutti i pacchetti del robot che tu non hai compaiono nell'elenco dei file da cancellare. Leggi l'elenco prima di rispondere sì, e non usare insieme `--yes`.

Dopo la copia, sul robot va ricompilato il workspace (`colcon build`) prima di riavviare il sistema.

## Fare un backup

```bash
./fit4med_backup.sh
```

Salva una copia di `/home/fit4med/fit4med_ws/src/` in `/home/fit4med/bkp/AAAAMMGG/HHMM/`. Le cartelle `.git` non vengono salvate: la storia dei repository è già sui PC e su GitHub.

- Se fai due backup nello stesso minuto, il secondo si chiama `HHMM_2`.
- Se la copia fallisce, la cartella incompleta viene cancellata, così nell'elenco compaiono solo backup completi.
- I backup non vengono mai cancellati automaticamente. Per liberare spazio, cancellali a mano da `/home/fit4med/bkp/`.

## Ripristinare un backup

```bash
./fit4med_restore.sh
```

Lo script:

1. elenca i backup, dal più recente, con data, ora e dimensione:

   ```bash
   Backups in /home/fit4med/bkp (newest first):
       1)  2026-09-25 14:30   180M
       2)  2026-09-24 09:15   178M
   ```

2. chiede il numero del backup da ripristinare (`q` per uscire senza fare nulla);
3. chiede conferma (risposta predefinita: no);
4. chiede se salvare prima lo stato attuale (risposta predefinita: sì). Conviene sempre dire sì, così puoi tornare indietro;
5. rende `/home/fit4med/fit4med_ws/src/` **identica** al backup scelto: i file che non sono nel backup vengono cancellati, quelli modificati vengono sovrascritti. Le cartelle `.git` del robot restano come sono.

Dopo il ripristino va ricompilato il workspace (`colcon build`) prima di riavviare il sistema.

## Dal tuo PC o direttamente sul robot

`fit4med_backup.sh` e `fit4med_restore.sh` funzionano in entrambi i modi:

- **Dal tuo PC:** agiscono sul robot tramite ssh. Sul robot non serve avere questi script.
- **Sul robot** (per esempio dopo `ssh fit4med@192.168.1.1`): agiscono direttamente sul robot.

Lo script capisce da solo dove si trova: se viene eseguito dall'utente `fit4med` e la cartella `/home/fit4med/fit4med_ws/src` esiste, lavora in locale; altrimenti si collega al robot. Per scegliere esplicitamente:

```bash
./fit4med_backup.sh --local          # agisce su questa macchina
./fit4med_backup.sh --host 192.168.1.1   # agisce sul robot tramite ssh
```

Lanciati dal tuo PC, i backup prendono il nome dall'orologio del tuo PC e non da quello del robot. Il robot non è collegato a internet e il suo orologio potrebbe essere sbagliato.

`fit4med_to_robot_sync.sh` invece si lancia sempre dal tuo PC.

## Da Windows

In `ps_scripts` ci sono gli stessi tre comandi per PowerShell. Fanno esattamente le stesse cose, con le stesse domande:

| Linux | Windows |
| --- | --- |
| `./fit4med_to_robot_sync.sh` | `.\ps_scripts\fmrr_to_robot_sync.ps1` |
| `./fit4med_backup.sh` | `.\ps_scripts\fmrr_backup.ps1` |
| `./fit4med_restore.sh` | `.\ps_scripts\fmrr_restore.ps1` |

Le opzioni hanno la forma di PowerShell:

| Linux | Windows |
| --- | --- |
| `--dry-run` | `-DryRun` |
| `--folder Fit4Med/rehab_gui` | `-Folder Fit4Med\rehab_gui` |
| `--backup` / `--no-backup` | `-Backup` / `-NoBackup` |
| `--yes` | `-Yes` |
| `--host <ip>` | `-RobotHost <ip>` |
| `--local-path <cartella>` | `-LocalPath <cartella>` |
| `--delete-extra-folders` | `-DeleteExtraFolders` |

Servono Windows 10 o 11, che includono già `ssh`, `scp` e `tar`. Su Windows non esiste `rsync`, quindi lo script impacchetta il software in un unico file, lo carica sul robot e lo allinea lì.

Per non dover scrivere la password ogni volta (`ssh-copy-id` non esiste su Windows):

```powershell
ssh-keygen          # solo se non hai ancora una chiave; premi Invio a ogni domanda
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh fit4med@192.168.1.1 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```

Senza chiave, su Windows la password viene chiesta a ogni passaggio (fino a 4-5 volte).

**Fine riga Windows.** Git per Windows di solito converte la fine delle righe in formato Windows (CRLF), che sul robot non va bene (gli script non partono). Il repository ora tiene la fine riga Linux (LF) per tutti i file (file `.gitattributes`). In ogni caso lo script di sincronizzazione converte in LF i file di testo con CRLF prima di copiarli sul robot, e lo segnala con `text file(s) with Windows line endings (CRLF): converted to LF`. Fanno eccezione gli script Windows (`.ps1`, `.bat`, `.cmd`).

Per mettere a posto una volta per tutte una copia del repository scaricata prima, dalla cartella del repository:

```powershell
git pull
git rm --cached -r -q .
git reset --hard
```

Attenzione: `git reset --hard` cancella le modifiche locali non salvate con commit.

## Esempio completo di aggiornamento

```bash
# 1. sul tuo PC, scarica la versione aggiornata del software
cd ~/fit4med_ws/src/Fit4Med
git pull

# 2. guarda cosa cambierebbe sul robot
cd bash_scripts
./fit4med_to_robot_sync.sh --dry-run

# 3. copia sul robot (rispondi "sì" alla domanda sul backup)
./fit4med_to_robot_sync.sh

# 4. sul robot, ricompila
ssh fit4med@192.168.1.1
cd ~/fit4med_ws && colcon build
```

Se dopo l'aggiornamento qualcosa non funziona, torna alla versione precedente con `./fit4med_restore.sh` e scegli il backup fatto al punto 3.

## Problemi frequenti

| Messaggio | Causa e soluzione |
| --- | --- |
| `Host not reachable` | Il PC non raggiunge `192.168.1.1`. Controlla il cavo di rete e l'indirizzo IP del tuo PC. |
| `SSH connection failed` | Password sbagliata, oppure ssh non attivo sul robot. |
| `rsync: command not found` | rsync non è installato sul tuo PC o sul robot. Sul robot, senza internet, va installato dal pacchetto `.deb`. |
| `No backups in /home/fit4med/bkp` | Non è ancora stato fatto nessun backup. |
| La password viene chiesta più volte | Su Linux è normale se passano più di 2 minuti tra un passaggio e l'altro; su Windows viene chiesta a ogni passaggio. Con una chiave ssh non viene più chiesta. |
| `is not a workspace 'src' folder` | La cartella da copiare non si chiama `src`. Indica quella giusta con `--local-path` (`-LocalPath` su Windows). |
| `text file(s) with Windows line endings (CRLF): converted to LF` | Non è un errore: la copia del repository sul PC Windows ha la fine riga Windows e lo script l'ha convertita. Per eliminare il messaggio vedi [Da Windows](#da-windows). |

## Permesso di leggere il journal di sistema (log EtherCAT)

L'archivio dei log di ogni accensione (`~/.ros/fit4med_log/run_NNNN_….zip`, vedi il README principale, passo 9), copiato sul PC della GUI da `fmrr_retrieve_logs.ps1`, include le righe di `ethercat.service` e i messaggi EtherCAT del kernel. Sono log di sistema: l'utente `fit4med` li può leggere solo se è nel gruppo `systemd-journal` (va bene anche `adm`). Senza questo gruppo le righe EtherCAT mancano.

**Verifica**, sul robot (`ssh fit4med@192.168.1.1`):

```bash
id fit4med
```

Nell'elenco `groups=` deve comparire `systemd-journal` (oppure `adm`), per esempio:

```bash
uid=1000(fit4med) gid=1000(fit4med) groups=1000(fit4med),4(adm),...,999(systemd-journal)
```

**Se manca**, aggiungilo:

```bash
sudo usermod -aG systemd-journal fit4med
```

Il gruppo vale solo per i processi avviati dopo:

- **sessione ssh:** basta riconnettersi;
- **servizio `fit4med-bringup@…`:** gira sotto il `systemd --user` di `fit4med`, che è già avviato con i gruppi vecchi. Riavvia il robot, oppure `sudo systemctl restart user@$(id -u fit4med).service` (ferma tutti i servizi utente di `fit4med`).

**Prova che la lettura funzioni**, come utente `fit4med`:

```bash
journalctl -u ethercat.service -b -n 5 --no-pager
```

Devono comparire righe di `ethercat.service`. Se il permesso manca compare invece `Hint: You are currently not seeing messages from other users and the system` oppure `No journal files were opened due to insufficient permissions`.

**Controllo sull'archivio di un'accensione:** apri `journal/fit4med_ethercat_timeline.log` nello zip. Se in testa c'è `# WARNING: fit4med cannot read the system journal`, il gruppo non era attivo per il servizio (tipicamente: `systemd --user` non ancora riavviato). Senza il warning e con righe `[ETHERCAT]`/`[KERNEL]` è tutto a posto.

## Diagnostica real-time: latenza e processi che rubano CPU al loop EtherCAT

Due script, da lanciare **sul robot come `fit4med`** (non con `sudo`: lo chiedono loro) **mentre il sistema lavora normalmente** (robot acceso, GUI collegata, terapia in corso). I risultati vanno in `~/.ros/fit4med_diagnostics/`.

| Script | Cosa fa |
| --- | --- |
| `rt_latency_test.sh [-d 10m]` | Misura con `cyclictest` quanto in ritardo si sveglia un thread real-time su ogni CPU. Riepilogo in `summary.txt`: latenza massima per CPU e numero di risvegli in ritardo oltre 250/500/1000/2000/4000 µs. Con un ciclo di 2–4 ms, oltre qualche centinaio di µs è un avviso, oltre 1000 µs il loop perde cicli. |
| `rt_cpu_hogs.sh [-d 600] [-a 1000]` | Nella stessa finestra raccoglie: configurazione real-time (kernel PREEMPT_RT, governor, irqbalance, limiti rtprio, priorità dei thread di `ros2_control_node` e degli IRQ della scheda EtherCAT), CPU e preemption per thread (`pidstat`), carico per CPU (`mpstat`), frame EtherCAT persi, warning del kernel, messaggi di overrun del `controller_manager` e, con `rtla timerlat`, l'analisi del kernel su **chi** ha bloccato la CPU al primo ritardo oltre `-a` µs. |

Pacchetti necessari (il robot è offline: scarica i `.deb` su un PC con internet): `rt-tests` (cyclictest), `sysstat` (pidstat, mpstat), `rtla` (o `linux-tools-$(uname -r)`; senza, `rt_cpu_hogs.sh` funziona ma non dice chi ha bloccato la CPU). Le righe del kernel richiedono il gruppo `systemd-journal` (sezione precedente).

Conviene lanciarli una volta a robot fermo e una nella condizione più pesante, e confrontare. Non lanciarli insieme: si disturbano a vicenda.

## Svuotare le cartelle dei log del robot

Sul robot i log si accumulano senza che nulla li cancelli. Questa procedura li elimina tutti e lascia il PC pulito. **Quello che cancelli non si recupera**: se potrebbero servire, copiali prima sul PC della GUI.

### Dove stanno

| Cartella | Contenuto |
| --- | --- |
| `~/.ros/fit4med_log/` | un `run_NNNN_….zip` per accensione; i vecchi `log_AAAAMMGG_HHMMSS.zip` di `log.sh`; `recover.log`; `.session_counter` (il contatore delle accensioni) |
| `~/.ros/log/` | log ROS non ancora archiviati (comandi `ros2` lanciati a mano, accensione in corso) |
| `/tmp/fit4med_snapshot.*` | copie temporanee di `fmrr_retrieve_logs.ps1` rimaste se il retrieve è stato interrotto |
| journal di sistema | `journalctl`: gestito da systemd, vedi il passo 5 |

Non sono log e **questa procedura non li tocca**: i backup del software (`~/bkp/`) e le registrazioni `ros2 bag` (`~/fit4med_ws/bag/`).

### 1. Copia sul PC della GUI (facoltativo)

Dal PC della GUI, con il robot acceso:

```powershell
.\ps_scripts\fmrr_retrieve_logs.ps1 -All
```

### 2. Ferma il robot

Dal PC della GUI: `.\ps_scripts\fmrr_cleanup.ps1`. Poi, sul robot (`ssh fit4med@192.168.1.1`), controlla che non giri più nulla:

```bash
systemctl --user list-units 'fit4med-bringup@*' --no-legend   # nessuna riga "active"
pgrep -af 'ros2|launch' || echo "nessun processo ROS"
```

Così l'accensione in corso viene chiusa e archiviata prima di essere cancellata, e nessun processo sta ancora scrivendo nelle cartelle.

### 3. Guarda quanto occupano

```bash
du -sh ~/.ros/fit4med_log ~/.ros/log /tmp/fit4med_snapshot.* 2> /dev/null
ls ~/.ros/fit4med_log | head -n 20
```

### 4. Cancella

**Tutto**, tranne il contatore delle accensioni:

```bash
find ~/.ros/fit4med_log -mindepth 1 -maxdepth 1 ! -name .session_counter -exec rm -rf {} +
rm -rf ~/.ros/log/* /tmp/fit4med_snapshot.*
```

Il contatore `.session_counter` va tenuto: la numerazione riprende dall'ultimo numero, e le copie già sul PC della GUI non si confondono con i nuovi archivi (un nuovo `run_0001` sarebbe un'altra accensione rispetto al `run_0001` già copiato).

**Oppure solo i vecchi**, tenendo le ultime 20 accensioni:

```bash
cd ~/.ros/fit4med_log
ls run_*.zip | sort -t _ -k 2,2n | head -n -20 | xargs -r rm -f --   # tiene le ultime 20
rm -f log_*.zip recover.log                                          # archivi di log.sh, pre-ottobre 2026
rm -rf ~/.ros/log/* /tmp/fit4med_snapshot.*
```

Controlla con `du -sh ~/.ros/fit4med_log ~/.ros/log`.

### 5. Journal di sistema (facoltativo)

Il journal di systemd ha già un limite di spazio, gestito da systemd. Ogni archivio `run_….zip` contiene già il journal della sua accensione. Per vedere quanto occupa e ridurlo:

```bash
journalctl --disk-usage
sudo journalctl --vacuum-time=30d     # tiene gli ultimi 30 giorni
```

Le accensioni non ancora archiviate perdono così il journal più vecchio di 30 giorni. Se servono, prima fai il passo 1.
