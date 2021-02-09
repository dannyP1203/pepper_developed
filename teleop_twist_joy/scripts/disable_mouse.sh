#!/bin/bash
id=`xinput list | grep -P "PLAYSTATION.*pointer" | grep -oP "id=\d+" | grep -oP "\d+"`
# props=`xinput list-props $id`
# # echo "$props"
# props_mouse=(`echo "$props" | grep "Generate Mouse Events" | grep -oP "\d+"`)
# props_keyboard=(`echo "$props" | grep "Generate Key Events" | grep -oP "\d+"`)
# # xinput set-prop $id ${props_mouse[0]} $[1-${props_mouse[1]}]
# xinput set-prop $id ${props_keyboard[0]} $[1-${props_mouse[1]}]

xinput disable $id

echo "Joystick as mouse disabled!"
exit 0
