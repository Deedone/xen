#!/bin/bash
# Stop immediately if any executed command has exit status different from 0.
set -e
set -o pipefail

usage() {
    echo "Usage: $0 SECTION_ID SECTION_NAME FILE EXIT_CODE" >&2
    exit 2
}

[ $# -eq 4 ] || usage

# Load settings and helpers
. "$(dirname "$0")/action.helpers"

# Absolute path of the ECLAIR bin directory.
export ECLAIR_BIN_DIR=/home/gitlab-runner/bugseng/eclair/bin/

# Directory where this script resides: usually in a directory named "ECLAIR".
SCRIPT_DIR="$(
  cd "$(dirname "$0")"
  echo "${PWD}"
)"
# Directory where to put all ECLAIR output and temporary files.
if [[ -z "${ECLAIR_OUTPUT_DIR:-}" ]]; then
  ECLAIR_OUTPUT_DIR="${PWD}/ECLAIR/out"
fi

ECLAIR_HTML_LOG="${ECLAIR_OUTPUT_DIR}/HTML.log"
mkdir -p "${ECLAIR_OUTPUT_DIR}/html"

# Create the HTML reports file.
PROJECT_ECD="${ECLAIR_OUTPUT_DIR}/PROJECT.ecd"
"${ECLAIR_BIN_DIR}eclair_report" \
  "-db='${PROJECT_ECD}'" \
  -quiet \
  "-full_html='${ECLAIR_OUTPUT_DIR}/html'" \
  > "${ECLAIR_HTML_LOG}" 2>&1

log_file "$@"
