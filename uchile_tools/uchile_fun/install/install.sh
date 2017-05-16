#!/bin/bash

# uchile_fun package installer
# run: $ bash $(rospack find uchile_fun)/install/install.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[uchile_fun]:${reset}"

echo "$installer Installing Instagram API"
sudo pip install instagram-python

sudo wget https://raw.githubusercontent.com/danleyb2/Instagram-API/master/InstagramAPI/src/http/devices.csv -O /usr/local/lib/python2.7/dist-packages/InstagramAPI/src/http/devices.csv

cat << EOF

Patches:
/usr/local/lib/python2.7/dist-packages/InstagramAPI/src/http/Response/ConfigureResponse.py
L13 self.image_url = response['media']['image_versions2']['candidates'][0]['url']

/usr/local/lib/python2.7/dist-packages/InstagramAPI/src/Instagram.py
L105 if not self.customPath:

EOF

