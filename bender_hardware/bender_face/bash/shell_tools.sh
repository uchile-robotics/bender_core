#!/bin/bash

# test face connectivity using ping (icmp protocol).
# a period  will be printed for each dropped package
#alias bender_test_head-icmp='ping  -i 1 -f $BENDER_HEAD_IP'


bender-set_emotion()
{
	rostopic pub --once /bender/face/head bender_msgs/Emotion "{Order: 'changeFace', Action: '$1', X: 0}"
}

bender-set_mouth_state()
{
	rostopic pub --once /bender/face/head bender_msgs/Emotion "{Order: 'changeFace', Action: '$1', X: 0}" 
}

bender-set_neck_yaw()
{
    local _angle

    _angle="$1"
	rostopic pub --once /bender/face/head bender_msgs/Emotion "{Order: 'MoveX', Action: '', X: $_angle}"
}

##############################################################################################
#   BASH AUTOCOMPLETE FOR PREVIOUS COMMANDS
##############################################################################################

if [  "$CATKIN_SHELL" != "bash" ]; then
    return
fi
# from now: using bash


_bendercomplete_bender-set_emotion()
{
    local cur prev opts opts1 opts2

    # available options + bender_ packages
    opts1="serious happy1 happy2 happy3 sad1 sad2 sad3 angry1 angry2 angry3 surprise ashamed"
    opts2="standby eyebrow greetings 1313 ear yes no lost relaxed normal agitated blink flirt"
    opts="${opts1} ${opts2}"

    # empty reply
    COMPREPLY=()

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"
  
    # only complete one option
    if [[ $COMP_CWORD == 1 ]] ; then

        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        return 0
    fi
}

_bendercomplete_bender-set_mouth_state()
{
    local cur prev opts

    # available options + bender_ packages
    opts="speakOn speakOff"
    opts="${opts}"

    # empty reply
    COMPREPLY=()

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"
  
    # only complete one option
    if [[ $COMP_CWORD == 1 ]] ; then

        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )     
        return 0
    fi
}

complete -F "_bendercomplete_bender-set_emotion" "bender-set_emotion"
complete -F "_bendercomplete_bender-set_mouth_state" "bender-set_mouth_state"