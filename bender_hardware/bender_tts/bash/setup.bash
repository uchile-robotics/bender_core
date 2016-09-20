#!/bin/bash

complete -F "_bendercomplete_str" "bender-say" "bender-say_eng" "bender-say_esp"
complete -F "_bendercomplete_NOT_COMPLETE" "bender-say_random_eng" "bender-say_random_esp"


# NOTE:
# COMP_WORDS is an array containing all individual words in the current command line.
# COMP_CWORD is an index of the word containing the current cursor position.
# COMPREPLY  is an array variable from which Bash reads the possible completions.
#
# compgen -W "${opts}" -- ${cur}
# returns the array of elements from "opts" matching the current word "${cur}"
#
_bendercomplete_bender_say_set_language()
{
    local cur opts

    # available options
    opts="spanish english"

    # empty reply
    COMPREPLY=()

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"
  
    # only complete one option
    if [ "$COMP_CWORD" = "1" ] ; then

        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        return 0
    fi
}
complete -F "_bendercomplete_bender_say_set_language" "bender-say_set_language"

