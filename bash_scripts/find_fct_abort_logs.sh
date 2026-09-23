#!/usr/bin/env bash

set -u

DEFAULT_LOG_DIR="/mnt/c/Users/NicolaPedrocchi/Downloads/fit4med_logs/fit4med_logs"
LOG_DIR="${1:-$DEFAULT_LOG_DIR}"

if [[ ! -d "$LOG_DIR" ]]; then
    printf 'Cartella non trovata: %s\n' "$LOG_DIR" >&2
    exit 1
fi

found_files=0
abort_files=0

while IFS= read -r -d '' source_file; do
    ((found_files++))

    if grep -aiq -- 'abort' "$source_file"; then
        source_name="${source_file##*/}"
        destination_name="${source_name/log_fct_valido_/log_fct_abort_}"
        destination="$LOG_DIR/$destination_name"
        cp -- "$source_file" "$destination"
        printf 'Trovato abort: %s -> %s\n' "$source_name" "$destination_name"
        ((abort_files++))
    fi
done < <(find "$LOG_DIR" -maxdepth 1 -type f \
    -regextype posix-extended \
    -regex '.*/log_fct_valido_[0-9]+_[0-9]+\.log' \
    -print0 | sort -z)

printf 'Scansione completata: %d file controllati, %d file copiati.\n' \
    "$found_files" "$abort_files"
