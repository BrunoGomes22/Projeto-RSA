#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

die() {
	local _ret=$2
	test -n "$_ret" || _ret=1
	test "$_PRINT_HELP" = yes && print_help >&2
	echo "$1" >&2
	exit ${_ret}
}

begins_with_short_option() {
	local first_option all_short_options='clh'
	first_option="${1:0:1}"
	test "$all_short_options" = "${all_short_options/$first_option/}" && return 1 || return 0
}

# THE DEFAULTS INITIALIZATION - OPTIONALS
_arg_config=
_arg_logs=
_arg_record=false
_arg_dev=false

print_help() {
	printf '%s\n' "Fleet Manager: Launch containers with ready-to-run ground station modules"
	printf 'Usage: %s [-c|--config <arg>] [-l|--logs <arg>] [-h|--help]\n' "$0"
	printf '\t%s\n' "-c, --config: specify applications.properties config file"
	printf '\t%s\n' "-l, --logs: specify directory to save logs and rosbags"
	printf '\t%s\n' "-r, --record, --no-record: records ROS2 data and server logs(false by default)"
	printf '\t%s\n' "-d, --dev, --no-dev: development mode, with tty attached and source files mounted (false by default)"
	printf '\t%s\n' "-h, --help: Prints help"
}

parse_commandline() {
	while test $# -gt 0; do
		_key="$1"
		case "$_key" in
		-c | --config)
			test $# -lt 2 && die "ERROR: Missing value for the optional argument '$_key'." 1
			_arg_config="$2"
			shift
			;;
		--config=*)
			_arg_config="${_key##--config=}"
			;;
		-c*)
			_arg_config="${_key##-c}"
			;;
		-l | --logs)
			test $# -lt 2 && die "ERROR: Missing value for the optional argument '$_key'." 1
			_arg_logs="$2"
			shift
			;;
		--logs=*)
			_arg_logs="${_key##--logs=}"
			;;
		-l*)
			_arg_logs="${_key##-l}"
			;;
		-r | --no-record | --record)
			_arg_record=true
			test "${1:0:5}" = "--no-" && _arg_record=false
			;;
		-r*)
			_arg_record="on"
			_next="${_key##-s}"
			if test -n "$_next" -a "$_next" != "$_key"; then
				{ begins_with_short_option "$_next" && shift && set -- "-s" "-${_next}" "$@"; } || die "The short option '$_key' can't be decomposed to ${_key:0:2} and -${_key:2}, because ${_key:0:2} doesn't accept value and '-${_key:2:1}' doesn't correspond to a short option."
			fi
			;;
		-d | --no-dev | --dev)
			_arg_dev=true
			test "${1:0:5}" = "--no-" && _arg_dev=false
			;;
		-d*)
			_arg_dev="on"
			_next="${_key##-s}"
			if test -n "$_next" -a "$_next" != "$_key"; then
				{ begins_with_short_option "$_next" && shift && set -- "-s" "-${_next}" "$@"; } || die "The short option '$_key' can't be decomposed to ${_key:0:2} and -${_key:2}, because ${_key:0:2} doesn't accept value and '-${_key:2:1}' doesn't correspond to a short option."
			fi
			;;
		-h | --help)
			print_help
			exit 0
			;;
		-h*)
			print_help
			exit 0
			;;
		*)
			_PRINT_HELP=yes die "FATAL ERROR: Got an unexpected argument '$1'" 1
			;;
		esac
		shift
	done
}

launch_container() {
	if [ ! -z $_arg_config ]; then
		_arg_config=$(readlink -f $_arg_config)
	else
		_arg_config=$SCRIPT_DIR/../configs/server.properties
	fi

	if [ ! -z $_arg_logs ]; then
		_arg_logs=$(readlink -f $_arg_logs)
	else
		if $_arg_record ; then
			default_dir=$SCRIPT_DIR/../logs/$(date +'%Y-%m-%d')
			[[ -d $default_dir ]] || mkdir -p $default_dir
			_arg_logs=$default_dir
		else
			[[ -d /tmp/fleetman ]] || mkdir /tmp/fleetman
			_arg_logs=/tmp/fleetman
		fi
	fi

	# Stops a previous instance of this container if it is running
	id=$(docker ps -a -q --filter ancestor="fleetman/gs" --format="{{.ID}}")
	if [ ${#id} -ne 0 ]; then
		docker stop $id
	fi

	if $_arg_dev ; then
		CFG_FILE=$_arg_config LOG_DIR=$_arg_logs docker compose -f $SCRIPT_DIR/docker-compose.override.yml -f $SCRIPT_DIR/docker-compose.yml up -d
		docker exec -it groundstation service ntp start
		docker exec -it groundstation bash
	else
		CFG_FILE=$_arg_config RECORD_BAGS=$_arg_record LOG_DIR=$_arg_logs docker compose -f $SCRIPT_DIR/docker-compose.prod.yml -f $SCRIPT_DIR/docker-compose.yml up -d
		CFG_FILE=$_arg_config RECORD_BAGS=$_arg_record LOG_DIR=$_arg_logs docker compose -f $SCRIPT_DIR/docker-compose.prod.yml -f $SCRIPT_DIR/docker-compose.yml logs -f groundstation
	fi
	CFG_FILE=$_arg_config RECORD_BAGS=$_arg_record LOG_DIR=$_arg_logs docker compose -f $SCRIPT_DIR/docker-compose.yml down
}

parse_commandline "$@"
launch_container
