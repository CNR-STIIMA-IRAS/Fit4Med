# Impostare la soglia "dati EtherCAT non validi" (`max_incomplete_wc_cycles`)

Questa guida spiega cos'è il parametro `max_incomplete_wc_cycles`, quando può
servire cambiarlo e come farlo, passo per passo.

**Di solito non va toccato.** Il valore normale è **5**, ed è quello giusto
per l'uso quotidiano.

---

## Cosa fa, in parole semplici

Il computer del robot parla con gli azionamenti (drive) degli assi X, Y, Z e con
il sensore di forza attraverso la rete EtherCAT, 250 volte al secondo: un
"giro" ogni 4 millesimi di secondo (4 ms). A ogni giro tutti i dispositivi
devono rispondere.

Se un cavo si stacca o si rompe, i dispositivi dopo il guasto smettono di
rispondere. L'azionamento rimasto isolato toglie subito la forza al motore, ma
il computer, senza risposte nuove, continuerebbe a vedere i dati vecchi
("motore acceso"). Per l'asse Z, che senza forza scende per gravità, è
pericoloso: il freno si chiuderebbe in ritardo.

Il parametro dice **dopo quanti giri consecutivi senza risposta completa** il
computer smette di fidarsi dei dati vecchi. A quel punto considera tutti gli
azionamenti in errore (FAULT): il freno si chiude e gli assi si fermano.

| Valore | Tempo prima dell'intervento | Note |
|---|---|---|
| 3 | 12 ms | più rapido, ma un disturbo breve può fermare la macchina |
| **5** | **20 ms** | **valore normale** |
| 10 | 40 ms | più tollerante, intervento più lento |
| 0 | mai | **controllo disattivato: NON usare** (il freno può chiudersi con secondi di ritardo) |

Un singolo giro perso ogni tanto è normale e non fa intervenire il controllo.

---

## Quando cambiarlo

Solo se **tutte** queste cose sono vere:

1. la macchina si ferma con gli azionamenti in errore senza un motivo visibile;
2. nei log c'è la riga `Domain: process data NOT valid` (vedi sotto come
   cercarla);
3. i cavi EtherCAT sono stati controllati e sono a posto.

Se i cavi hanno un problema, **non alzare il valore**: la macchina si ferma
perché la rete si interrompe davvero, e alzare il valore ritarda solo la
chiusura del freno. Va riparato il cavo.

Nel dubbio, prima di cambiare qualcosa chiedi a Nicola.

---

## Cosa serve

- Il **PC Windows con la GUI**, collegato al robot.
- Il robot **acceso**, con la piattaforma **ferma** (nessun esercizio in corso).
- La password dell'utente `fit4med` del robot, se viene chiesta.

---

## Passo 1 — Collegarsi al robot

In **PowerShell** sul PC Windows:

```powershell
cd C:\fit4med_ws\src\Fit4Med
.\ps_scripts\fmrr_ssh.ps1
```

Il prompt diventa qualcosa come `fit4med@fit4med-cnr:~$`: da qui i comandi
vanno al robot.

## Passo 2 — Vedere il valore attuale

```bash
grep -n max_incomplete_wc_cycles ~/fit4med_ws/src/Fit4Med/tecnobody_workbench/urdf/ros2_control/platform_complete.ros2_control.urdf
```

Risponde con una riga come:

```
15:            <param name="max_incomplete_wc_cycles">5</param>
```

Il numero tra `>` e `<` è il valore attuale (qui 5).

## Passo 3 — Cambiare il valore

Sostituisci `NUOVO` con il valore voluto (per esempio `8`) e scrivi:

```bash
sed -i 's|<param name="max_incomplete_wc_cycles">[0-9]*</param>|<param name="max_incomplete_wc_cycles">NUOVO</param>|' ~/fit4med_ws/src/Fit4Med/tecnobody_workbench/urdf/ros2_control/platform_complete.ros2_control.urdf
```

Ripeti il comando del passo 2 e controlla che compaia il numero nuovo.

## Passo 4 — Rendere effettiva la modifica

```bash
cd ~/fit4med_ws && colcon build --packages-select tecnobody_workbench
```

Deve finire con `Summary: 1 package finished`. Poi chiudi la finestra con
`exit`, riavvia la piattaforma dal pendant come al solito e portala in
funzione.

## Passo 5 — Controllare che il valore sia stato preso

Riapri la finestra del robot (passo 1) e scrivi:

```bash
journalctl --user -u 'fit4med-bringup@*' --since "15 min ago" --no-pager | grep -E "Process data not valid after|max_incomplete_wc_cycles"
```

Deve comparire una riga con il tuo numero, per esempio:

```
Process data not valid after 8 consecutive cycles with an incomplete working counter.
```

Se compare invece `Invalid max_incomplete_wc_cycles`, il valore scritto non è
un numero intero valido: la piattaforma non parte. Torna al passo 3 e scrivi un
numero intero (5 è il valore normale).

Se compare `max_incomplete_wc_cycles = 0`, il controllo è disattivato: rimetti
subito un valore diverso da 0.

Chiudi con `exit`.

---

## Importante: avvisa Nicola

La modifica fatta così vale **solo sul robot**. Il prossimo aggiornamento del
software dal PC di sviluppo riporta il valore scritto nel repository (oggi 5).
Se il valore nuovo deve restare, comunica a Nicola il numero e il motivo, così
viene salvato anche nel repository.

## Cercare nei log se il controllo è intervenuto

Recupera i log come al solito (`.\ps_scripts\fmrr_retrieve_logs.ps1`). Nella
cartella della partenza (`starts\NNN_platform_…\console.log`) cerca:

- `Domain: process data NOT valid, WC … incomplete for … cycles` — il
  controllo è intervenuto: gli azionamenti sono stati messi in errore e il
  freno chiuso;
- `Domain: process data valid again` — la rete è tornata completa.

Annota l'ora e manda gli archivi `run_….zip` a Nicola.
