#!/usr/bin/env bash

set -u

DEFAULT_LOG_DIR="/mnt/c/Users/NicolaPedrocchi/Downloads/fit4med_logs/fit4med_logs"
LOG_DIR="${1:-$DEFAULT_LOG_DIR}"

if [[ ! -d "$LOG_DIR" ]]; then
    printf 'Cartella non trovata: %s\n' "$LOG_DIR" >&2
    exit 1
fi

valid_index=1
while [[ -e "$LOG_DIR/log_valido_$valid_index" ]]; do
    ((valid_index++))
done

while IFS= read -r -d '' archive; do
    temporary_dir=$(mktemp -d "$LOG_DIR/.log_scan_tmp.XXXXXX")
    printf 'Analizzo: %s\n' "$archive"

    if ! unzip -q "$archive" -d "$temporary_dir"; then
        printf '  Archivio non valido, rimuovo la cartella temporanea.\n' >&2
        rm -rf "$temporary_dir"
        continue
    fi

    if grep -RIl --binary-files=without-match 'fct_manager_node' "$temporary_dir" >/dev/null 2>&1; then
        destination="$LOG_DIR/log_valido_$valid_index"
        while [[ -e "$destination" ]]; do
            ((valid_index++))
            destination="$LOG_DIR/log_valido_$valid_index"
        done
        mv "$temporary_dir" "$destination"
        printf '  Trovato: salvata in %s\n' "$destination"

        file_index=1
        while IFS= read -r -d '' matching_file; do
            extension=""
            if [[ "$matching_file" == *.* ]]; then
                extension=".${matching_file##*.}"
            fi

            output_file="$LOG_DIR/log_fct_valido_${valid_index}_${file_index}${extension}"
            while [[ -e "$output_file" ]]; do
                ((file_index++))
                output_file="$LOG_DIR/log_fct_valido_${valid_index}_${file_index}${extension}"
            done
            cp -- "$matching_file" "$output_file"
            printf '  File trovato: salvato in %s\n' "$output_file"
            ((file_index++))
        done < <(find "$destination" -type f -print0 | while IFS= read -r -d '' file; do
            if grep -Iq 'fct_manager_node' "$file"; then
                printf '%s\0' "$file"
            fi
        done)

        ((valid_index++))
    else
        printf '  Nessuna occorrenza, rimuovo la cartella temporanea.\n'
        rm -rf "$temporary_dir"
    fi
done < <(find "$LOG_DIR" -maxdepth 1 -type f -iname '*.zip' -print0 | sort -z)

printf 'Scansione completata.\n'