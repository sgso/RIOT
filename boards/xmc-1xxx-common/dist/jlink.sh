#!/bin/sh
# Global environment variables used:
# JLINK_IF:            JLink -if parameter
# JLINK_DEV:           JLink -device parameter
# JLINK_SPEED:         JLink -speed parameter

# The script supports the following actions:
#
# flash:        flash a given bin file
#               binfile is expected in binary format and is pointed to
#               by BINFILE environment variable
#
# debug:        starts JLinkGDBServer as GDB server in the background and
#               connects to the server with the GDB client specified by
#               the board (DBG environment variable)
#
#               options:
#               GDB_PORT:       port opened for GDB connections
#               TUI:            if TUI!=null, the -tui option will be used
#               ELFFILE:        path to the ELF file to debug
#
# debug-server: starts JLinkGDBServer as GDB server, but does not connect to
#               to it with any frontend. This might be useful when using
#               IDEs.
#
#               options:
#               GDB_PORT:       port opened for GDB connections
#
# reset:        triggers a hardware reset of the target board
#
#
# @author       Sebastian Sontberg <sebastian@sontberg.de>

# default GDB port
if [ -z "${GDB_PORT}" ]; then
    GDB_PORT=2331
fi

# default JLinkGDBServer command
if [ -z "${JLINK_GDB}" ]; then
    JLINK_GDB=JLinkGDBServer
fi
# default JLinkGDBExe command
if [ -z "${JLINK_EXE}" ]; then
    JLINK_EXE=JLinkExe
fi

# address for loadbin command
if [ -z "${LOADBIN_ADDR}" ]; then
    LOADBIN_ADDR=0
fi

# GDB tui
if [ -n "${TUI}" ]; then
    TUI=-tui
fi


test_binfile() {
    if [ ! -f "${BINFILE}" ]; then
        echo "Error: Unable to locate BINFILE"
        echo "       (${BINFILE})"
        exit 1
    fi
}

do_flash() {
    test_binfile

    if [ -n "${PRE_FLASH_CHECK_SCRIPT}" ]; then
        sh -c "${PRE_FLASH_CHECK_SCRIPT} '${BINFILE}'"
        RETVAL=$?
        if [ $RETVAL -ne 0 ]; then
            echo "pre-flash checks failed, status=$RETVAL"
            exit $RETVAL
        fi
    fi

    # flash device
    sh -c "${JLINK_EXE} ${JLINK_OPTS} ${JLINK_EXTRA_OPTS}" <<EOF
loadbin $BINFILE $BIN_ADDR
r
qc
EOF
    echo ""
}

do_debug() {
    # matching debug-server running?
    pgrep -f "${JLINK_GDB} ${JLINK_OPTS} -port ${GDB_PORT}"

    if [ $? -eq 0 ];then
        ${DBG} ${TUI} -ex "target extended-remote localhost:${GDB_PORT}" ${ELFFILE}
    else
        echo "No debug server found. Use 'make debug-server'"
    fi
}

do_debugserver() {
    sh -c "${JLINK_GDB} ${JLINK_OPTS} -port ${GDB_PORT} ${JLINK_EXTRA_OPTS}"
}

do_reset() {
    sh -c "${JLINK_EXE} ${JLINK_OPTS} ${JLINK_EXTRA_OPTS}" <<EOF
r
g
qc
EOF
}

#
# parameter dispatching
#
ACTION="$1"
shift # pop $1 from $@

case "${ACTION}" in
  flash)
    echo "### Flashing Target ###"
    do_flash "$@"
    ;;
  debug)
    echo "### Starting Debugging ###"
    do_debug "$@"
    ;;
  debug-server)
    echo "### Starting GDB Server ###"
    do_debugserver "$@"
    ;;
  reset)
    echo "### Resetting Target ###"
    do_reset "$@"
    ;;
  *)
    echo "Usage: $0 {flash|debug|debug-server|reset}"
    ;;
esac
