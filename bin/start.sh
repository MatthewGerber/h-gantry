#!/bin/sh

cd /home/matthewgerber/Repos/h-gantry || exit
/usr/bin/nohup /home/matthewgerber/.local/bin/poetry run flask --app h_gantry.sand_art_flask run --host 0.0.0.0 &
