#!/bin/sh

# TODO: hacer que el gui_subtitles no muestre tildes así: "canci'on", sino que así: "canción".
# TODO: gui_subtitles con mapper para palabras que suenan bien si las escriben feo:
#   - e.g: "feisbuc" vs. "facebook"
#   - e.g: "b'ender" vs. "bender"


# TODO: idioma (ideas)
#   - servicio para cambiar idioma del sintetizador
#   - agregar parámetro "language" al servicio
#   - agregar nuevos servicios para cada idioma
return 0

# Useful data
export BENDER_TTS_SAY_LAST_RAND=-1


# TOOLS
# --------------------------------------------------------------------

bender-say_set_language() {
    rosservice call /bender/hw/tts/set_language "data: '$1'"
}

bender-say() {
    rosservice call /bender/hw/tts/say "text: '$1'" 
}

bender-say_esp() {
    bender-set_language spanish
    bender-say "$1"
}

bender-say_eng() {
    bender-set_language english
    bender-say "$1"
}


bender-say_random_esp() {

    local _THIS_DIR n_phrases rand_idx

    # load phrases
    _THIS_DIR="$(rospack find bender_tts)/shell"
    . "$_THIS_DIR"/phrases.bash

    # compute random index
    n_phrases=${#phrases_esp[@]}
    rand_idx=$(hexdump -n 1 -e '/2 "%u"' /dev/urandom)
    rand_idx=$(( rand_idx % n_phrases ));

    # don't repeat phrases
    if [ "$rand_idx" = "$BENDER_TTS_SAY_LAST_RAND" ]; then
        rand_idx=$(( rand_idx + 1  ));
        rand_idx=$(( rand_idx % n_phrases  ));
    fi

    # store idx
    export BENDER_TTS_SAY_LAST_RAND="$rand_idx"

    # synthesize
    echo "say: -. ${phrases_esp[$rand_idx]} .- "
    bender-say "${phrases_esp[$rand_idx]}"
}

bender-say_random_eng() {

    local _THIS_DIR n_phrases rand_idx
    
    # load phrases
    _THIS_DIR="$(rospack find bender_tts)/shell"
    . "$_THIS_DIR"/phrases.bash

    # compute random index
    n_phrases=${#phrases_eng[@]}
    rand_idx=$(hexdump -n 1 -e '/2 "%u"' /dev/urandom)
    rand_idx=$(( rand_idx % n_phrases ));

    # don't repeat phrases
    if [ "$rand_idx" = "$BENDER_TTS_SAY_LAST_RAND" ]; then
        rand_idx=$(( rand_idx + 1  ));
        rand_idx=$(( rand_idx % n_phrases  ));
    fi

    # store idx
    export BENDER_TTS_SAY_LAST_RAND="$rand_idx"

    # synthesize
    echo "say: -. ${phrases_eng[$rand_idx]} .- "
    bender-say "${phrases_eng[$rand_idx]}"
}


##############################################################################################
#   BASH AUTOCOMPLETE FOR PREVIOUS COMMANDS
##############################################################################################

if [  "$CATKIN_SHELL" = "bash" ]; then
    # from now: using bash
    _THIS_DIR="$(rospack find bender_tts)/shell"
    . "$_THIS_DIR"/setup.bash
fi

