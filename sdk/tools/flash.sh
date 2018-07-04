#!/bin/bash

CURRENT_DIR=`pwd`
SCRIPT_NAME=`readlink -e "${BASH_SOURCE[0]}"`
SCRIPT_DIR=`dirname "$SCRIPT_NAME"`

# function   : show_help
# description: Show help and exit.
function show_help()
{
	echo "  Usage:"
	echo "       $0 [-c <UART Poart> -b <UART Baudrate>] <(e)spk file> [<(e)spk file> ...]"
	echo ""
	echo "  Mandatory argument:"
	echo "       (e)spk file path"
	echo ""
	echo "  Optional arguments:"
	echo "       -c: Serial port (default: /dev/ttyUSB0)"
	echo "       -b: Serial baudrate (default: 115200)"
	echo "       -e: Extract loader archive"
	echo "       -l: Flash loader"
	echo "       -h: Show help (This message)"
	exit
}

# Option handler
# -b: UART Baudrate (default: 115200)
# -c: UART Port (default: /dev/ttyUSB0)
# -h: Show Help
UART_BAUDRATE="115200"
UART_PORT="/dev/ttyUSB0"
UPDATE_ZIP=""
LOADR_PATH=""
while getopts b:c:s:e:l:h OPT
do
	case $OPT in
		'b' ) UART_BAUDRATE=$OPTARG;;
		'c' ) UART_PORT=$OPTARG;;
		'e' ) UPDATE_ZIP=$OPTARG;;
		'l' ) LOADR_PATH=$OPTARG;;
		'h' ) show_help;;
	esac
done

# Shift argument position after option
shift $(($OPTIND - 1))

# Pickup spk and espk files
ESPK_FILES=""
SPK_FILES=""

for arg in $@
do
	if [ "`echo ${arg} | grep "\.spk$"`" ]; then
		SPK_FILES="${SPK_FILES} ${arg}"
	elif [ "`echo ${arg} | grep "\.espk$"`" ]; then
		ESPK_FILES="${ESPK_FILES} -S ${arg}"
	fi
done

if [ "${UPDATE_ZIP}" != "" ]; then
	${SCRIPT_DIR}/eula.py -i ${UPDATE_ZIP}
	exit
fi

# Check loader version
${SCRIPT_DIR}/eula.py -c

if [ "${LOADR_PATH}" != "" ]; then
	ESPK_FILES="`find ${LOADR_PATH} -name "*.espk"`"
fi

if [ "${SPK_FILES}${ESPK_FILES}" == "" ]; then
	echo "ERROR: No (e)spk files are contains."
	echo ""
	show_help
fi

# Get platform type
case "$(uname -s)" in
	Linux*)
		PLATFORM=linux
		;;
	Darwin*)
		PLATFORM=macosx
		;;
	CYGWIN*|MINGW32*|MSYS*)
		PLATFORM=windows
		;;
	*)
		echo "ERROR: Unknown platform"
		echo ""
		show_help
		;;
esac

# Flash into spresense board
${SCRIPT_DIR}/${PLATFORM}/flash_writer -s -c ${UART_PORT} -d -b ${UART_BAUDRATE} -n ${ESPK_FILES} ${SPK_FILES}
