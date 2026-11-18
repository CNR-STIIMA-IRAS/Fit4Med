# Attivare i log diagnostici sul robot

Questa guida spiega come installare sul robot due "registratori" che lavorano in
silenzio mentre la macchina viene usata normalmente:

- **monitor EtherCAT**: annota se un azionamento (drive) perde la comunicazione,
  i contatori di errore della rete e gli allarmi dei drive;
- **monitor del carico**: annota quanto lavora il computer del robot.

Servono a capire perché a volte il robot si blocca. Una volta attivati non
richiedono niente: i dati finiscono da soli negli archivi dei log, che si
recuperano come al solito con `fmrr_retrieve_logs.ps1`.

L'operazione si fa **una volta sola** e richiede circa 15 minuti.

---

## Cosa serve

- Il **PC Windows con la GUI** (il teach pendant), collegato al robot.
- Il robot **acceso**.
- **Git** installato sul PC (è lo stesso usato per installare la GUI).
- L'accesso al repository `https://github.com/CNR-STIIMA-IRAS/Fit4Med` (se GitHub
  chiede di accedere, usa l'account che ti è stato indicato).
- La password dell'utente `fit4med` del robot, se viene chiesta.

> I comandi vanno scritti in **PowerShell** (Start → digita "PowerShell" →
> Windows PowerShell), tranne quelli del passo 4, che vanno scritti nella
> finestra collegata al robot. Copia e incolla i comandi esattamente come sono.

---

## Passo 1 — Scaricare il codice (una volta)

Il codice va in una cartella **nuova**, diversa da quella della GUI.
**Non toccare la cartella della GUI** (di solito `C:\Fit4Med`).

In PowerShell:

```powershell
mkdir C:\fit4med_ws\src
cd C:\fit4med_ws\src
git clone -b rt-cpu-layout https://github.com/CNR-STIIMA-IRAS/Fit4Med.git
```

Alla fine deve esistere la cartella `C:\fit4med_ws\src\Fit4Med`.

> **Se la cartella esiste già** (lo hai fatto in passato), aggiornala invece di
> scaricarla di nuovo:
>
> ```powershell
> cd C:\fit4med_ws\src\Fit4Med
> git fetch
> git checkout rt-cpu-layout
> git pull
> ```

> **Se il PC non è collegato a Internet:** da un altro PC apri la pagina del
> repository su GitHub, scegli il branch `rt-cpu-layout`, poi
> **Code → Download ZIP**. Copia lo ZIP sul PC (per esempio con una chiavetta),
> estrailo in `C:\fit4med_ws\src\` e **rinomina** la cartella estratta
> (`Fit4Med-rt-cpu-layout`) in `Fit4Med`.

> **Il nome della cartella `src` è importante:** lo script del passo 3 si
> rifiuta di copiare se la cartella non si chiama esattamente `src`.

---

## Passo 2 — Fermare la piattaforma

Prima di copiare qualcosa sul robot, la piattaforma deve essere **ferma**.
Con la macchina in stato sicuro, chiudi tutto come a fine giornata:

```powershell
cd C:\fit4med_ws\src\Fit4Med
.\ps_scripts\fmrr_cleanup.ps1
```

> Se PowerShell risponde che l'esecuzione degli script è disabilitata, esegui
> una volta `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, rispondi
> `S` (o `Y`) e ripeti il comando.

---

## Passo 3 — Copiare gli script sul robot

Si copia **solo** la cartella `bash_scripts`: il resto del software del robot
non viene toccato.

**3a. Prova (non cambia niente):**

```powershell
cd C:\fit4med_ws\src\Fit4Med
.\ps_scripts\fmrr_to_robot_sync.ps1 -Folder Fit4Med/bash_scripts -DryRun
```

Lo script mostra cosa cambierebbe. Devono comparire **solo** questi file
(l'ordine non conta):

- `launch_ros2_env.sh` (modificato)
- `ec_link_monitor.sh` (nuovo)
- `rt_load_monitor.sh` (nuovo)
- `ec_delta_monitor_setup.sh` (nuovo)
- `fit4med_diag_monitors.sh` (nuovo)

Può comparire anche "restore executable permission" per alcuni di questi: va
bene. **Se compaiono altri file, o file da cancellare, fermati** e contatta
Nicola, mandandogli quello che vedi a schermo.

**3b. Copia vera:**

```powershell
.\ps_scripts\fmrr_to_robot_sync.ps1 -Folder Fit4Med/bash_scripts
```

- Alla domanda *"Create a backup of the robot sources before the sync?"*
  rispondi **`y`** (fa una copia di sicurezza del software del robot).
- Alle eventuali richieste di conferma rispondi **`y`**.
- Alla fine deve comparire un messaggio di completamento senza `[ERROR]`.

Non serve compilare niente.

---

## Passo 4 — Attivare i log

Apri una finestra collegata al robot:

```powershell
cd C:\fit4med_ws\src\Fit4Med
.\ps_scripts\fmrr_ssh.ps1
```

(Se chiede la password, è quella dell'utente `fit4med`.) Il prompt cambia in
qualcosa come `fit4med@fit4med-cnr:~$`: ora i comandi vanno al robot.

Scrivi:

```bash
~/fit4med_ws/src/Fit4Med/bash_scripts/fit4med_diag_monitors.sh enable
```

Deve rispondere:

```
pendant (systemd unit): on
shell (~/.bashrc):      on
=> monitors ENABLED (from the next start of the stack)
```

Poi chiudi la finestra del robot con:

```bash
exit
```

I log si attivano **dal prossimo avvio** della piattaforma, sia quando la
avvii dal pendant (come di solito), sia quando qualcuno la avvia da terminale.

---

## Passo 5 — Controllare che funzioni (una volta)

1. Avvia la piattaforma **come fai di solito** e portala in funzione
   (piattaforma pronta, GUI collegata).
2. Aspetta un minuto, poi apri di nuovo la finestra del robot
   (`.\ps_scripts\fmrr_ssh.ps1`) e scrivi:

   ```bash
   pgrep -fa "ec_link_monitor|rt_load_monitor" | grep -v pgrep
   ```

   Devono comparire **due righe**, una con `ec_link_monitor.sh` e una con
   `rt_load_monitor.sh`. Se non compare niente, contatta Nicola.
3. Chiudi con `exit`. Da qui in poi usa la macchina normalmente.

---

## Ogni giorno: niente da fare

I log si registrano da soli. Si fermano quando la piattaforma si ferma (anche
dopo un fungo di emergenza) e ripartono al riavvio successivo.

## Quando il robot si blocca

1. **Annota l'ora** e cosa stava succedendo (esercizio in corso, movimento,
   asse che si è fermato, messaggi sulla GUI). Nota: l'orologio del pendant e
   quello del robot possono differire di qualche minuto; basta l'ora del
   pendant, indicando che è quella.
2. Continua pure a lavorare (riavvio, chiave, ecc.) come al solito.
3. A fine giornata, o quando ti viene chiesto, recupera i log dalla cartella
   della GUI come sempre:

   ```powershell
   cd C:\Fit4Med
   .\ps_scripts\fmrr_retrieve_logs.ps1 -Last 5
   ```

   I file arrivano in `Desktop\fit4med_logs\`. Manda a Nicola gli archivi
   `run_….zip` del giorno del problema, insieme agli appunti del punto 1.

   (Se la cartella della GUI non è `C:\Fit4Med`, usa quella dove la avvii di
   solito; funziona anche da `C:\fit4med_ws\src\Fit4Med`.)

---

## Disattivare i log

Quando ti verrà detto che non servono più:

```powershell
cd C:\fit4med_ws\src\Fit4Med
.\ps_scripts\fmrr_ssh.ps1
```

e nella finestra del robot:

```bash
~/fit4med_ws/src/Fit4Med/bash_scripts/fit4med_diag_monitors.sh disable
exit
```

Deve rispondere `=> monitors DISABLED`. Vale dal prossimo avvio.

Per sapere in qualunque momento se sono attivi:
`~/fit4med_ws/src/Fit4Med/bash_scripts/fit4med_diag_monitors.sh status`.

---

## Problemi frequenti

| Messaggio | Cosa fare |
|---|---|
| `git` non è riconosciuto | Git non è installato su questo PC: contatta Nicola. |
| GitHub chiede utente e password, oppure "Repository not found" | Serve l'accesso al repository: contatta Nicola. |
| `is not a workspace 'src' folder` (passo 3) | Il codice non è in `C:\fit4med_ws\src\Fit4Med`. Ricontrolla il passo 1 (la cartella deve chiamarsi `src`). |
| `Host not reachable` / `Connection timed out` | Il PC non vede il robot: controlla che il robot sia acceso e il cavo di rete collegato, poi riprova. |
| Il passo 3a mostra file diversi da quelli elencati | Non fare il passo 3b. Manda a Nicola una foto o una copia dello schermo. |
| `Permission denied` al passo 4 | Lo script non è stato copiato correttamente: ripeti il passo 3b, poi il passo 4. |
| Il passo 5 non mostra due righe | Verifica con `fit4med_diag_monitors.sh status` che risulti `ENABLED`, riavvia la piattaforma e riprova. Se ancora niente, contatta Nicola. |

I log **non cambiano il funzionamento della macchina**: se qualcosa va storto
con questa procedura, la macchina continua a funzionare come prima.
