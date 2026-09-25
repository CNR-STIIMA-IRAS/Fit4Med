# Aggiornamento, backup e ripristino del software del robot

Questi tre script servono a portare il software del controllore sul robot, a salvarne una copia e a tornare a una copia precedente. Esistono per Linux (in questa cartella) e per Windows (in `ps_scripts`, vedi [Da Windows](#da-windows)):

| Script | Cosa fa |
|---|---|
| `fit4med_to_robot_sync.sh` | Copia sul robot il software che hai sul tuo PC. Prima propone un backup. |
| `fit4med_backup.sh` | Salva una copia del software del robot. |
| `fit4med_restore.sh` | Elenca le copie salvate e ne ripristina una. |

Il robot (`192.168.1.1`) non è collegato a internet. Il software aggiornato si scarica sul tuo PC, che è collegato a internet, e da lì si copia sul robot con `fit4med_to_robot_sync.sh`.

## Dove si trovano le cose

| Cosa | Dove |
|---|---|
| Software sul tuo PC | la cartella `src` del workspace che contiene la tua copia del repository `Fit4Med` (vedi sotto) |
| Software sul robot | `/home/fit4med/fit4med_ws/src/` |
| Backup sul robot | `/home/fit4med/bkp/AAAAMMGG/HHMM/`, per esempio `/home/fit4med/bkp/20260925/1430/` |

**Il tuo workspace può stare dove vuoi.** Lo script usa la cartella `src` che contiene la copia del repository da cui lo lanci. Per esempio, se il repository è in `/home/mario/lavoro/fit4med_ws/src/Fit4Med`, copia `/home/mario/lavoro/fit4med_ws/src/`. Per usare un'altra cartella: `--local-path <percorso della cartella src>`.

Per sicurezza la cartella deve chiamarsi `src`. Se hai clonato il repository fuori da un workspace (per esempio in `~/codice/Fit4Med`), lo script si ferma: altrimenti copierebbe sul robot tutto il contenuto di `~/codice` e cancellerebbe dal robot i pacchetti che lì non ci sono.

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

Il robot diventa **identico** alla copia sul tuo PC. Le modifiche fatte direttamente sul robot vengono sovrascritte.

Lo script:
1. controlla che il robot risponda e che ssh funzioni;
2. chiede se fare un backup del software attuale del robot (risposta predefinita: sì). Se il backup fallisce, si ferma senza toccare nulla;
3. se sul robot ci sono file che sul tuo PC non esistono, li elenca e chiede se cancellarli (risposta predefinita: no). Se rispondi no, si ferma e il robot resta com'era;
4. copia i file.

Le cartelle `.git` e le cache di Python (`__pycache__`, `*.pyc`) sul robot non vengono né sovrascritte né cancellate.

**Per vedere cosa cambierebbe senza modificare nulla:**
```bash
./fit4med_to_robot_sync.sh --dry-run
```
Le righe `*deleting` sono i file che verrebbero cancellati dal robot, le altre quelli che verrebbero copiati. In questa modalità non si fa nessun backup.

**Opzioni utili:**

| Opzione | Effetto |
|---|---|
| `--dry-run` | Mostra cosa cambierebbe, senza modificare nulla. |
| `--folder <cartella>` | Copia solo una cartella, relativa a `~/fit4med_ws/src/`, per esempio `--folder Fit4Med/rehab_gui`. Anche le cancellazioni restano dentro quella cartella. |
| `--backup` | Fa il backup senza chiedere. |
| `--no-backup` | Non fa il backup e non chiede. |
| `--yes` | Cancella i file presenti solo sul robot senza chiedere conferma. |
| `--host <ip>` | Usa un altro indirizzo del robot. |
| `--local-path <cartella>` | Usa un'altra cartella `src` del tuo PC. |

> **Attenzione:** senza `--folder` viene copiata tutta la cartella `src` del tuo PC. Se sul robot c'è un pacchetto che tu non hai, comparirà nell'elenco dei file da cancellare. In quel caso rispondi **no** e verifica prima di procedere.

Dopo la copia, sul robot va ricompilato il workspace (`colcon build`) prima di riavviare il sistema.

## Fare un backup

```bash
./fit4med_backup.sh
```

Salva una copia completa di `/home/fit4med/fit4med_ws/src/` in `/home/fit4med/bkp/AAAAMMGG/HHMM/`.

- Se fai due backup nello stesso minuto, il secondo si chiama `HHMM_2`.
- Se la copia fallisce, la cartella incompleta viene cancellata, così nell'elenco compaiono solo backup completi.
- I backup non vengono mai cancellati automaticamente. Per liberare spazio, cancellali a mano da `/home/fit4med/bkp/`.

## Ripristinare un backup

```bash
./fit4med_restore.sh
```

Lo script:
1. elenca i backup, dal più recente, con data, ora e dimensione:
   ```
   Backups in /home/fit4med/bkp (newest first):
       1)  2026-09-25 14:30   180M
       2)  2026-09-24 09:15   178M
   ```
2. chiede il numero del backup da ripristinare (`q` per uscire senza fare nulla);
3. chiede conferma (risposta predefinita: no);
4. chiede se salvare prima lo stato attuale (risposta predefinita: sì). Conviene sempre dire sì, così puoi tornare indietro;
5. rende `/home/fit4med/fit4med_ws/src/` **identica** al backup scelto: i file che non sono nel backup vengono cancellati, quelli modificati vengono sovrascritti.

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
|---|---|
| `./fit4med_to_robot_sync.sh` | `.\ps_scripts\fmrr_to_robot_sync.ps1` |
| `./fit4med_backup.sh` | `.\ps_scripts\fmrr_backup.ps1` |
| `./fit4med_restore.sh` | `.\ps_scripts\fmrr_restore.ps1` |

Le opzioni hanno la forma di PowerShell:

| Linux | Windows |
|---|---|
| `--dry-run` | `-DryRun` |
| `--folder Fit4Med/rehab_gui` | `-Folder Fit4Med\rehab_gui` |
| `--backup` / `--no-backup` | `-Backup` / `-NoBackup` |
| `--yes` | `-Yes` |
| `--host <ip>` | `-RobotHost <ip>` |
| `--local-path <cartella>` | `-LocalPath <cartella>` |

Servono Windows 10 o 11, che includono già `ssh`, `scp` e `tar`. Su Windows non esiste `rsync`, quindi lo script impacchetta il software in un unico file, lo carica sul robot e lo allinea lì.

Per non dover scrivere la password ogni volta (`ssh-copy-id` non esiste su Windows):
```powershell
ssh-keygen          # solo se non hai ancora una chiave; premi Invio a ogni domanda
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh fit4med@192.168.1.1 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```
Senza chiave, su Windows la password viene chiesta a ogni passaggio (fino a 4-5 volte).

**Fine riga Windows.** Git per Windows di solito converte la fine delle righe in formato Windows (CRLF). Uno script `.sh` convertito così non funziona sul robot. Il repository ora lo impedisce per i `.sh` (file `.gitattributes`), ma una copia scaricata prima va sistemata una volta, dalla cartella del repository:
```powershell
git pull
git rm --cached -r -q .
git reset --hard
```
Attenzione: `git reset --hard` cancella le modifiche locali non salvate con commit. Se il problema è presente, lo script di sincronizzazione se ne accorge, elenca i file e si ferma senza toccare il robot.

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
|---|---|
| `Host not reachable` | Il PC non raggiunge `192.168.1.1`. Controlla il cavo di rete e l'indirizzo IP del tuo PC. |
| `SSH connection failed` | Password sbagliata, oppure ssh non attivo sul robot. |
| `rsync: command not found` | rsync non è installato sul tuo PC o sul robot. Sul robot, senza internet, va installato dal pacchetto `.deb`. |
| `No backups in /home/fit4med/bkp` | Non è ancora stato fatto nessun backup. |
| La password viene chiesta più volte | Su Linux è normale se passano più di 2 minuti tra un passaggio e l'altro; su Windows viene chiesta a ogni passaggio. Con una chiave ssh non viene più chiesta. |
| `is not a workspace 'src' folder` | La cartella da copiare non si chiama `src`. Indica quella giusta con `--local-path` (`-LocalPath` su Windows). |
| `shell script(s) have Windows line endings (CRLF)` | Copia del repository fatta su Windows con la conversione della fine riga. Vedi [Da Windows](#da-windows). |
