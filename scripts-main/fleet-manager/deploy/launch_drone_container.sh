#!/bin/bash

die() {
	local _ret=$2
	test -n "$_ret" || _ret=1
	test "$_PRINT_HELP" = yes && print_help >&2
	echo "$1" >&2
	exit ${_ret}
}

begins_with_short_option() {
	local first_option all_short_options='csh'
	first_option="${1:0:1}"
	test "$all_short_options" = "${all_short_options/$first_option/}" && return 1 || return 0
}

# THE DEFAULTS INITIALIZATION - POSITIONALS
_positionals=()
# THE DEFAULTS INITIALIZATION - OPTIONALS
_arg_config=
_arg_serial=false
_arg_bridge=false

print_help() {
	printf '%s\n' "Fleet Manager: Launch container with ready-to-run drone module"
	printf 'Usage: %s [-c|--config <arg>] [-s|--(no-)serial] [-h|--help] <drone-id>\n' "$0"
	printf '\t%s\n' "<drone-id>: specify drone id"
	printf '\t%s\n' "-c, --config: specify config file"
	printf '\t%s\n' "-s, --serial, --no-serial: connect through serial port (false by default)"
	printf '\t%s\n' "-b, --bridge, --no-bridge: launch ROS2/1 bridge (false by default)"
	printf '\t%s\n' "-h, --help: Prints help"
}

parse_commandline() {
	_positionals_count=0
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
		-s | --no-serial | --serial)
			_arg_serial=true
			test "${1:0:5}" = "--no-" && _arg_serial=false
			;;
		-s*)
			_arg_serial="on"
			_next="${_key##-s}"
			if test -n "$_next" -a "$_next" != "$_key"; then
				{ begins_with_short_option "$_next" && shift && set -- "-s" "-${_next}" "$@"; } || die "The short option '$_key' can't be decomposed to ${_key:0:2} and -${_key:2}, because ${_key:0:2} doesn't accept value and '-${_key:2:1}' doesn't correspond to a short option."
			fi
			;;
		-b | --no-bridge | --bridge)
			_arg_bridge=true
			test "${1:0:5}" = "--no-" && _arg_bridge=false
			;;
		-b*)
			_arg_bridge="on"
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
			_last_positional="$1"
			_positionals+=("$_last_positional")
			_positionals_count=$((_positionals_count + 1))
			;;
		esac
		shift
	done
}

handle_passed_args_count() {
	local _required_args_string="'drone-id'"
	test "${_positionals_count}" -ge 1 || _PRINT_HELP=yes die "FATAL ERROR: Not enough positional arguments - we require exactly 1 (namely: $_required_args_string), but got only ${_positionals_count}." 1
	test "${_positionals_count}" -le 1 || _PRINT_HELP=yes die "FATAL ERROR: There were spurious positional arguments --- we expect exactly 1 (namely: $_required_args_string), but got ${_positionals_count} (the last one was: '${_last_positional}')." 1
}

assign_positional_args() {
	local _positional_name _shift_for=$1
	_positional_names="_arg_drone_id "

	shift "$_shift_for"
	for _positional_name in ${_positional_names}; do
		test $# -gt 0 || break
		eval "$_positional_name=\${1}" || die "Error during argument parsing." 1
		shift
	done
}

find_port() {
	while read line; do
		IFS=' ' read -r -a split <<<"$line"
		if [ "${split[0]}" == "port:" ]; then
			IFS='"' read -r -a port <<<"${split[1]}"
			if [ ${#port[1]} == 0 ]; then
				port=${port[0]}
			else
				port=${port[1]}
			fi
			IFS='://' read -r -a protocol <<<"$port"
			if [ "${protocol}" == "serial" ]; then
				port=${port:9}
			elif [ "${protocol}" == "tcp" ] || [ "${protocol}" == "udp" ]; then
				port=
			else
				die "ERROR: Invalid port protocol. Must be either tcp, udp or serial."
			fi
		fi
	done <"$_arg_config"
}

launch_container() {
	if [ ! -z $_arg_config ]; then
		cfg_var="-e CFG_FILE="$(basename $_arg_config)""
		cfg_flag=" -v ${PWD}/$_arg_config:/ws/"$(basename $_arg_config)""
		find_port
		if [ ${#port} != 0 ]
		then
			if [[ -d $port ]]; then
				echo "FATAL: $port is a directory. Delete before launching."
		       	exit
		    elif [[ -e $port ]]; then
				port_flag="--privileged -v $port:$port"
			else
				echo "FATAL: $port does not exist. Is the flight controller connected?"
				exit
			fi
		fi
	else
		if $_arg_serial; then
			cfg_var="-e CFG_FILE=drone_cfg_serial.yml"
		else
			cfg_var="-e CFG_FILE=drone_cfg_udp.yml"
		fi
	fi

	if [ $_arg_bridge = true ]; then
		docker run -d --network host --rm --name ros_bridge fleetman/ros_bridge
		echo "Launched ROS2/1 bridge"
	fi

	# Stops a previous instance of this container if it is running
	id=$(docker ps -a -q --filter ancestor="fleetman/drone" --format="{{.ID}}")
	if [ ${#id} -ne 0 ]; then
		docker stop $id
	fi

	docker run \
		--rm \
		--network host \
		-v /etc/localtime:/etc/localtime:ro \
		-v /dev:/dev \
		-e DRONE_ID=$_arg_drone_id \
		--name $_arg_drone_id \
		$port_flag $cfg_var $cfg_flag \
		-it fleetman/drone

	if [ $_arg_bridge = true ]; then
		docker stop ros_bridge
	fi
}

parse_commandline "$@"
handle_passed_args_count
assign_positional_args 1 "${_positionals[@]}"
launch_container
