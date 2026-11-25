# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

<#
.SYNOPSIS
    Raccoglie in un unico ZIP gli archivi del robot e i log locali della GUI.

.DESCRIPTION
    Crea uno ZIP fit4med_logs_YYYYMMDD-HHMMSS-fff.zip per ogni recupero, in
    Desktop\fit4med_logs sotto il profilo dell'utente Windows. Copia lo stesso
    ZIP in ~/.ros/fit4med_log sul PC del robot.
    Dentro lo ZIP, robot contiene gli archivi originali del robot e gui
    contiene i log correnti e i backup ruotati della GUI. Il file clocks.txt
    alla radice riporta l'ora di entrambi i sistemi al momento del recupero.

    Per ogni bring-up concluso, fit4med_session_log.sh prepara sul robot un
    archivio ~/.ros/fit4med_log/run_NNNN_YYYYMMDD-HHMMSS.zip. Lo ZIP include
    session.txt con i dati della sessione, i log ROS e i journal relativi
    all'intervallo di quel bring-up.

    Per ciascun bring-up ancora in corso, lo script remoto crea una copia
    temporanea run_NNNN_..._partial.zip con i log raccolti fino a quel momento,
    mentre il bring-up continua. I parziali vengono scaricati a ogni recupero.
    Per i definitivi si riutilizzano, quando disponibili, le copie sul PC
    lasciate dalle versioni precedenti o incluse negli ZIP di raccolta.

.PARAMETER GuiIp
    Parametro accettato per permettere l'uso dei collegamenti esistenti.
    Il valore passato viene ignorato. L'indirizzo SSH viene da $remoteHost.

.PARAMETER GuiLogDir
    Cartella locale in cui la GUI scrive i log. Il percorso predefinito
    corrisponde a C:\temp.

.PARAMETER Last
    Numero di archivi definitivi da selezionare, da 1 a 100000. Per impostazione
    predefinita seleziona 3 archivi. La selezione segue il progressivo NNNN.

.PARAMETER All
    Seleziona tutti gli archivi definitivi e ha precedenza su Last.
    Le copie delle sessioni in corso si aggiungono in entrambi i casi.

.EXAMPLE
    .\fmrr_retrieve_logs.ps1
    Recupera gli ultimi tre archivi definitivi e le eventuali sessioni in corso.

.EXAMPLE
    .\fmrr_retrieve_logs.ps1 -Last 10
    Recupera gli ultimi dieci archivi definitivi e le eventuali sessioni in corso.

.EXAMPLE
    .\fmrr_retrieve_logs.ps1 -All
    Recupera tutti gli archivi definitivi e le eventuali sessioni in corso.

.NOTES
    Servono ssh e scp sul PC e l'accesso SSH all'account fit4med sul robot.
    Per raccogliere i journal EtherCAT, l'account deve poter leggere i log
    di sistema, ad esempio tramite il gruppo systemd-journal. Per la procedura
    di configurazione, vedi bash_scripts/README.md.
#>

param(
    [string]$GuiIp = "",
    [string]$GuiLogDir = "C:\temp",
    [ValidateRange(1, 100000)]
    [int]$Last = 3,
    [switch]$All
)

# Destinazione SSH/SCP, script Bash sul robot e cartella di raccolta sul PC.
$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$robot = "${remoteUser}@${remoteHost}"
$robotScript = "/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/fit4med_session_log.sh"
$robotLogDir = ".ros/fit4med_log"
$localDestination = Join-Path $env:USERPROFILE "Desktop\fit4med_logs"

# Entrambi i comandi remoti, snapshot e list, restituiscono una riga per ZIP
# nel formato percorso<TAB>ora di avvio<TAB>motivo di fine. Il percorso Linux
# serve a SCP. L'ultimo componente diventa il nome del file locale. I campi
# descrittivi possono essere vuoti e vengono usati nel riepilogo.
function ConvertFrom-FmrrArchiveLine([string]$Line) {
    $fields = $Line -split "`t"
    [pscustomobject]@{
        Path   = $fields[0]
        Name   = ($fields[0] -split '/')[-1]
        Start  = if ($fields.Count -gt 1) { $fields[1] } else { "" }
        Reason = if ($fields.Count -gt 2) { $fields[2] } else { "" }
    }
}

# Riusa gli archivi definitivi delle raccolte precedenti, cercandoli per nome.
# La tabella contiene solo gli archivi richiesti da list e ancora da copiare.
function Restore-FmrrRobotArchives($Pending, [string]$Destination, [string]$Source) {
    foreach ($bundle in (Get-ChildItem -LiteralPath $Source -File -Filter "fit4med_logs_*.zip" |
                         Sort-Object Name -Descending)) {
        if ($Pending.Count -eq 0) { break }
        try {
            $previous = [System.IO.Compression.ZipFile]::OpenRead($bundle.FullName)
            try {
                foreach ($name in @($Pending.Keys)) {
                    $entry = $previous.GetEntry("robot/$name")
                    if ($null -eq $entry) { $entry = $previous.GetEntry("robot\$name") }
                    if ($null -eq $entry) { continue }

                    $target = Join-Path $Destination $name
                    [System.IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $target, $true)
                    $Pending.Remove($name)
                }
            }
            finally {
                $previous.Dispose()
            }
        }
        catch {
            Write-Warning "Could not reuse archives from $($bundle.Name): $_"
        }
    }
}

# Crea una directory di lavoro distinta per ogni esecuzione. Lo ZIP in
# costruzione resta fuori dalla cartella da comprimere e viene spostato nella
# destinazione solo dopo la chiusura. La directory di lavoro viene rimossa.
Add-Type -AssemblyName System.IO.Compression.FileSystem -ErrorAction Stop
$workDirectory = Join-Path $localDestination (".retrieve-" + [guid]::NewGuid().ToString("N"))
$payload = Join-Path $workDirectory "payload"
$robotLogs = Join-Path $payload "robot"
$partials = @()

try {
    [System.IO.Directory]::CreateDirectory($robotLogs) | Out-Null

    # Le letture sono consecutive. La differenza stima lo scarto tra gli
    # orologi al momento del recupero e include la latenza della chiamata SSH.
    $robotNow = ssh -T $robot "date --iso-8601=ns"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Could not read the robot clock (exit code $LASTEXITCODE)."
    }
    @(
        "Clocks at log retrieval (compare to align robot and GUI logs):",
        "  robot   : $robotNow",
        "  this PC : $(Get-Date -Format o)"
    ) | Out-File -FilePath (Join-Path $payload "clocks.txt") -Encoding utf8 -ErrorAction Stop

    # Snapshot archivia prima i bring-up lasciati aperti da un processo
    # terminato e poi prepara gli ZIP temporanei delle sessioni ancora attive.
    # Si chiama prima di list per includere anche gli archivi appena chiusi.
    Write-Output "Pack the bring-up still running (if any) on the robot ..."
    $partials = @(ssh -T $robot "bash $robotScript snapshot" | Where-Object { $_ } |
                  ForEach-Object { ConvertFrom-FmrrArchiveLine $_ })
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Could not prepare robot snapshots (exit code $LASTEXITCODE)."
    }

    # Il limite riguarda gli archivi definitivi. Per lo script Bash, list 0
    # significa tutti. L'ordinamento si basa sul progressivo del bring-up.
    $count = if ($All) { 0 } else { $Last }
    $archives = @(ssh -T $robot "bash $robotScript list $count" | Where-Object { $_ } |
                  ForEach-Object { ConvertFrom-FmrrArchiveLine $_ })
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Could not list robot archives (exit code $LASTEXITCODE)."
    }

    # Usa anche gli ZIP definitivi che le versioni precedenti lasciavano sul
    # Desktop. Le copie di lavoro finiscono tutte nella cartella robot.
    $pending = [ordered]@{}
    foreach ($archive in $archives) {
        $localArchive = Join-Path $localDestination $archive.Name
        if (Test-Path -LiteralPath $localArchive -PathType Leaf) {
            Copy-Item -LiteralPath $localArchive -Destination $robotLogs -ErrorAction Stop
        }
        else {
            $pending[$archive.Name] = $archive
        }
    }
    if ($pending.Count -gt 0) {
        Restore-FmrrRobotArchives $pending $robotLogs $localDestination
    }

    # Scarica i definitivi mancanti e tutti i parziali, che cambiano mentre
    # il robot lavora. Rimuovi eventuali file incompleti lasciati da SCP.
    foreach ($archive in (@($pending.Values) + $partials)) {
        $target = Join-Path $robotLogs $archive.Name
        scp -q "${robot}:$($archive.Path)" "$target"
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "Copy failed: $($archive.Path)"
            if (Test-Path -LiteralPath $target -PathType Leaf) {
                Remove-Item -LiteralPath $target -ErrorAction Stop
            }
        }
    }

    # La copia include la sessione GUI corrente e i backup datati disponibili
    # sul PC al momento del recupero, rispettando i nomi originali.
    $guiLogs = @(foreach ($pattern in "pyqt_hang*.log", "gui_errors*.log", "gui_console*.log") {
        Get-ChildItem -LiteralPath $GuiLogDir -File -Filter $pattern -ErrorAction SilentlyContinue
    })
    if ($guiLogs.Count -gt 0) {
        $guiStage = Join-Path $payload "gui"
        [System.IO.Directory]::CreateDirectory($guiStage) | Out-Null
        $guiLogs | Copy-Item -Destination $guiStage -ErrorAction Stop
        Write-Output "Included $($guiLogs.Count) GUI log(s) from $GuiLogDir"
    }
    else {
        Write-Warning "No GUI log found in $GuiLogDir."
    }

    $included = @(
        ($archives + $partials) | Where-Object {
            Test-Path -LiteralPath (Join-Path $robotLogs $_.Name) -PathType Leaf
        }
    )
    if ($included.Count -gt 0) {
        $included | Format-Table -AutoSize -Wrap Name, Start, @{ Label = "How it ended"; Expression = { $_.Reason } } | Out-String | Write-Output
    }
    else {
        Write-Warning "No robot archive included."
    }

    # Il nome datato conserva le raccolte precedenti, anche quando contengono
    # file GUI con gli stessi nomi. Il passaggio finale avviene sullo stesso
    # volume. Lo ZIP compare nella destinazione solo a scrittura conclusa.
    $temporaryZip = Join-Path $workDirectory "collection.zip"
    [System.IO.Compression.ZipFile]::CreateFromDirectory(
        $payload, $temporaryZip, [System.IO.Compression.CompressionLevel]::Fastest, $false
    )
    $zipPath = Join-Path $localDestination ("fit4med_logs_{0}.zip" -f (Get-Date -Format "yyyyMMdd-HHmmss-fff"))
    [System.IO.File]::Move($temporaryZip, $zipPath)
    Write-Output "Log archive: $zipPath"

    # Pubblica la copia sul robot solo dopo che SCP ha terminato il trasferimento.
    $robotZip = "$robotLogDir/$(Split-Path -Leaf $zipPath)"
    $robotTemporaryZip = "$robotLogDir/$(Split-Path -Leaf $workDirectory).tmp"
    ssh -T $robot "mkdir -p -- '$robotLogDir'"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Could not create the robot log directory ~/$robotLogDir (SSH exit code $LASTEXITCODE)."
    }
    else {
        try {
            scp -q "$zipPath" "${robot}:$robotTemporaryZip"
            if ($LASTEXITCODE -ne 0) { throw "SCP exit code $LASTEXITCODE" }

            ssh -T $robot "mv -nT -- '$robotTemporaryZip' '$robotZip' && test ! -e '$robotTemporaryZip'"
            if ($LASTEXITCODE -ne 0) { throw "Could not publish ~/$robotZip (SSH exit code $LASTEXITCODE)" }

            Write-Output "Log archive on the robot: ~/$robotZip"
        }
        catch {
            Write-Warning "Could not copy the log archive to the robot: $_"
            ssh -T $robot "rm -f -- '$robotTemporaryZip'"
        }
    }
}
finally {
    # Il percorso deve essere uno ZIP direttamente sotto una cartella
    # temporanea creata da snapshot. Gli archivi definitivi restano sul robot.
    try {
        foreach ($partial in $partials) {
            if ($partial.Path -match '^(/tmp/fit4med_snapshot\.[A-Za-z0-9]+)/[^/]+\.zip$') {
                $remoteTmp = $Matches[1]
                ssh -T $robot "rm -rf -- '$remoteTmp'"
                if ($LASTEXITCODE -ne 0) {
                    Write-Warning "Could not remove remote snapshot directory $remoteTmp."
                }
            }
        }
    }
    finally {
        if (Test-Path -LiteralPath $workDirectory -PathType Container) {
            Remove-Item -LiteralPath $workDirectory -Recurse -Force
        }
    }
}
