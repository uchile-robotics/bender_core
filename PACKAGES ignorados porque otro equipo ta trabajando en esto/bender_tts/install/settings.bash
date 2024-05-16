#!/bin/bash

## ----------------------------------------------------------------------------
## Useful Variables
## ----------------------------------------------------------------------------

# package name
pkg_name="bender_tts"

# package path
pkg_path=$(rospack find "$pkg_name")

# downloader script from mega
megadown_exe="$UCHILE_SYSTEM"/shell/megadown/megadown

## ----------------------------------------------------------------------------
## User Configuration 
## ----------------------------------------------------------------------------

# where download and mantain the files
install_space="$UCHILE_WS"/deps/bender/tts

# mark as installed 
install_token="$install_space"/INSTALLED

# compressed link
megadown_files='https://mega.nz/#!ChEVmTwZ!91ZVIyS3dezpZOPdw1R90noCZMrGab0UPFlr3RgZyM4'
megadown_tmp="$install_space"/.megadown

# compresed file
tarfile="$install_space"/mbrola_voices.tar.gz

# uncompresed file
tarfolder="$install_space"/mbrola_voices

# mbrola
mbrola_folder="$install_space"/mbrola_voices


