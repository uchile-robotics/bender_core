#!/bin/sh

bender-hwcheck_green () 
{
    local RED YELLOW GREEN NC port
    # Red          0;31     Light Red     1;31
    # Green        0;32     Light Green   1;32
    # Brown/Orange 0;33     Yellow        1;33
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    NC='\033[0m'

    if   _bender_check_if_bash ; then
        shopt -s nullglob
    elif _bender_check_if_zsh  ; then
        setopt null_glob
    fi

    # base
    port=/dev/bender/base
    if [ -r "$port" ]; then
        printf "> ${GREEN}[OK] base: port found at $port${NC}\n"
    elif [ -e "$port" ]; then
        printf "> ${RED}[FAIL] base: port found, but unreadable: $port${NC}\n"
    else
        printf "> ${YELLOW}[WARN] base: port not found $port${NC}\n"
    fi
    
    # laser front
    port=/dev/bender/sensors/hokuyo_H1311689
    if [ -r "$port" ]; then
        printf "> ${GREEN}[OK] laser front: port found  $port${NC}\n"
    elif [ -e "$port" ]; then
        printf "> ${RED}[FAIL] laser front: port found, but unreadable: $port${NC}\n"
    else
        printf "> ${YELLOW}[WARN] laser front: port not found $port ... looking for valid ttyACM*${NC}\n"
    fi
    for a_port in /dev/ttyACM[0-9]
    do   
        if [ "$(/opt/bender/udev/getID $a_port q)" = "$port" ]; then
            printf "   - ${YELLOW} found valid laser at $a_port . Should be redirected.. ${NC}\n"
            break
        else
            printf "   - $a_port: invalid\n"      
        fi
    done

    # laser rear
    port=/dev/bender/sensors/hokuyo_H0903381
    if [ -r "$port" ]; then
        printf "> ${GREEN}[OK] laser rear: port found  $port${NC}\n"
    elif [ -e "$port" ]; then
        printf "> ${RED}[FAIL] laser rear: port found, but unreadable: $port${NC}\n"
    else
        printf "> ${YELLOW}[WARN] laser rear: port not found $port ... looking for valid ttyACM*${NC}\n"
    fi
    for a_port in /dev/ttyACM[0-9]
    do
        if [ "$(/opt/bender/udev/getID $a_port q)" = "$port" ]; then
            printf "   - ${YELLOW} found valid laser at $a_port . Should be redirected.. ${NC}\n"
            break
        else
            printf "   - $a_port: invalid\n"      
        fi
    done
}
    
bender-hwcheck_gray () 
{
    # microphone

    # head

    # arms
    python $BENDER_WS/base_ws/src/bender_hardware/bender_arm/scripts/hw_check.py
}