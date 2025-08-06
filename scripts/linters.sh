#!/bin/bash

if [ "$#" -lt 2 ]; then 
    echo "Usage: $0 <lint_action> <lint_dir> ..."
    exit 1 
fi

BUILD_DIR="build"
LINT_ACTION="$1"
shift

FILE_EXTS=("h" "hpp" "c" "cpp")

function clang_tidy {
    FILES=("$@")
    if [ "${#FILES[@]}" -gt 0 ]; then
        clang-tidy -p "$BUILD_DIR" "${FILES[@]}"
    fi
}

function clang_format {
    FILES=("$@")
    if [ "${#FILES[@]}" -gt 0 ]; then
        clang-format -i "${FILES[@]}"
    fi
}

for FORMAT_DIR in "$@"; do
    for EXT in "${FILE_EXTS[@]}"; do
        mapfile -d '' FILES < <(find "$FORMAT_DIR" -type f -iname "*.$EXT" -print0)

        case "$LINT_ACTION" in
            tidy) clang_tidy "${FILES[@]}" ;;
            format) clang_format "${FILES[@]}" ;;
            *) echo "Invalid action: $LINT_ACTION"; exit 1 ;;
        esac
    done
done
