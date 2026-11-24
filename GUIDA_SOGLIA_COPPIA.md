# Soglia di coppia degli assi (protezione contro il "trascinamento")

Questa guida spiega la protezione che ferma la piattaforma quando un asse deve
spingere troppo, per esempio perché la persona attaccata alla manopola perde
l'equilibrio o resiste al movimento. Spiega anche come regolarla insieme al
fisioterapista.

---

## Cosa fa, in parole semplici

Ogni motore (assi X, Y, Z) sa quanta forza sta usando. Durante gli esercizi
normali usa poca forza: circa il 15–25% della sua forza "nominale".

Se un motore deve spingere **più della soglia** (oggi il **100%** della forza
nominale) per **più di un certo tempo** (oggi **100 millisecondi**, un decimo
di secondo), l'azionamento:

1. va in allarme (**AL030**) e toglie forza al motore;
2. il sistema chiude il freno e apre la catena di emergenza, come se fosse
   stato premuto il fungo;
3. per ripartire serve **girare la chiave**, come dopo un'emergenza.

C'è anche un **tetto** (oggi il **200%**): durante quel decimo di secondo il
motore non può comunque spingere più di così.

| Parametro | Cosa regola | Valore attuale | Unità |
|---|---|---|---|
| P1-57 | soglia di forza | 100 | % della forza nominale |
| P1-58 | per quanto tempo deve essere superata | 100 | millisecondi |
| 0x6072 | tetto massimo | 2000 | millesimi (2000 = 200%) |

Le tre impostazioni sono uguali per X, Y e Z e vengono scritte negli
azionamenti a ogni avvio della piattaforma.

---

## Come capire se è intervenuta

Sulla GUI la macchina si ferma come dopo un'emergenza. Nei log (recuperati come
sempre con `.\ps_scripts\fmrr_retrieve_logs.ps1`), nella cartella della
partenza `starts\NNN_platform_…`:

- **`ec_link_monitor_….log`**: nello storico allarmi dell'asse compare
  **`AL30(7121h)`** come allarme più recente.
- **`fault_snapshot_….csv`**: le prime righe dicono quale asse è andato in
  errore e la **forza massima** nei 2 secondi prima
  (`peak |effort| … joint_y=-1350 at -0.05 s`: 1350 millesimi = 135%).
  Il resto del file ha l'andamento della forza degli ultimi 20 secondi.

Annota l'ora e cosa stava facendo la persona: serve per decidere se la soglia è
giusta.

---

## I dati per decidere: `torque_stats.txt`

Nella cartella di ogni partenza (`starts\NNN_platform_…`) c'è il file
`torque_stats.txt`: per **ogni movimento** (ogni ripetizione di un esercizio e
ogni ritorno al punto di partenza), poi per **ogni esercizio** e per **tutta la
partenza**, la forza usata da ciascun asse, **in % della forza nominale**: la
stessa unità della soglia P1-57, quindi i numeri si confrontano direttamente.

```
repetition 2: 11:26:01-11:26:13 (12.0 s), SUCCEEDED
  joint_y: max  48.0% (-48.0% at +7.2 s)  mean  12.1%  p99  35.0%  over 100%: 0 ms (longest 0 ms)
== EXERCISE 1 (repetitions 1-3): 11:25:48-11:26:27 (39.0 s)
```

- **max**: la forza più alta (con segno, e dopo quanti secondi dall'inizio del
  movimento);
- **mean**: la forza media;
- **p99**: la forza superata solo nell'1% del tempo (meno sensibile a un picco
  isolato);
- **over 100%**: per quanto tempo la forza è stata sopra la soglia attuale
  (P1-57), in totale e nel tratto più lungo; se il tratto più lungo supera il
  tempo P1-58, la protezione è scattata.

Per scegliere la soglia con il fisioterapista: guarda il **max** degli esercizi
normali, con persone diverse. La soglia deve stare **chiaramente sopra** quei
valori (per esempio almeno 1,5–2 volte), e **sotto** la forza vista quando la
persona è stata davvero trascinata.

---

## Quando cambiarla

Va decisa **con il fisioterapista**:

- **la macchina si ferma troppo spesso** durante esercizi normali, senza che la
  persona sia in difficoltà → si può alzare un po' la soglia (P1-57) o il
  tempo (P1-58);
- **la macchina non si è fermata** quando avrebbe dovuto (persona trascinata) →
  si abbassa la soglia o il tempo.

Cambia un valore alla volta, a piccoli passi (per esempio 10–20% sulla soglia,
50 ms sul tempo), e prova di nuovo.

**Regole da rispettare sempre:**

1. P1-57 **mai 0**: con 0 la protezione è spenta.
2. Il tetto 0x6072 deve restare **sopra** la soglia: in millesimi, almeno
   `P1-57 × 10 + 500` (con P1-57 = 100: almeno 1500; oggi è 2000). Se il tetto
   è sotto la soglia, il motore non la raggiunge mai e la protezione non scatta.
3. Stessi valori per X, Y e Z, salvo decisione diversa presa con Nicola.

---

## Come cambiarla

Serve il PC Windows con la GUI, il robot acceso e la piattaforma **ferma**.

### Passo 1 — Collegarsi al robot

In PowerShell:

```powershell
cd C:\fit4med_ws\src\Fit4Med
.\ps_scripts\fmrr_ssh.ps1
```

Il prompt diventa `fit4med@fit4med-cnr:~$`.

### Passo 2 — Vedere i valori attuali

```bash
cd ~/fit4med_ws/src/Fit4Med/tecnobody_workbench/config
grep -H -E "index: 0x(2139|213a|6072)" delta400x.yaml delta400y.yaml delta750.yaml
```

Per ogni file compaiono tre righe; il numero dopo `value:` è il valore:
`0x2139` = soglia P1-57, `0x213a` = tempo P1-58, `0x6072` = tetto.

### Passo 3 — Cambiare un valore

Sostituisci `NUOVO` con il numero voluto. Soglia (P1-57), per esempio 120:

```bash
sed -i -E 's/(index: 0x2139, sub_index: 0, type: int16, value: )[0-9]+/\1NUOVO/' delta400x.yaml delta400y.yaml delta750.yaml
```

Tempo (P1-58), per esempio 150:

```bash
sed -i -E 's/(index: 0x213a, sub_index: 0, type: int16, value: )[0-9]+/\1NUOVO/' delta400x.yaml delta400y.yaml delta750.yaml
```

Tetto (0x6072), solo se serve per rispettare la regola 2:

```bash
sed -i -E 's/(index: 0x6072, sub_index: 0, type: uint16, value: )[0-9]+/\1NUOVO/' delta400x.yaml delta400y.yaml delta750.yaml
```

Ripeti il passo 2 e controlla i numeri nuovi in **tutti e tre** i file.

### Passo 4 — Rendere effettiva la modifica

```bash
cd ~/fit4med_ws && colcon build --packages-select tecnobody_workbench
```

Deve finire con `Summary: 1 package finished`. Chiudi con `exit`, riavvia la
piattaforma dal pendant e portala in funzione.

### Passo 5 — Controllare che gli azionamenti abbiano preso i valori

Riapri la finestra del robot (passo 1) e scrivi:

```bash
grep -h "settings slave" $(ls -t ~/.ros/fit4med_log/run_*/starts/*_platform_*/ec_link_monitor_*.log | head -1)
```

Per ogni asse deve comparire una riga con i tuoi valori, per esempio
`crash_P1-57=120% crash_P1-58=150ms max_torque(0x6072)=2000permille`.
Se compare `?` invece di un numero, o i valori sono diversi, contatta Nicola.

Chiudi con `exit`.

---

## Importante: avvisa Nicola

La modifica fatta così vale **solo sul robot**: il prossimo aggiornamento del
software la sovrascrive con i valori del repository. Comunica a Nicola i valori
decisi e il motivo, così vengono salvati anche nel repository.

## Nota

Questa protezione è una funzione dell'azionamento Delta ("Motor Crash
Protection"); non sostituisce il fungo di emergenza e la catena di sicurezza,
che restano la protezione principale.
