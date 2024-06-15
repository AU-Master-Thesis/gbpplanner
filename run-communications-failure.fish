#!/usr/bin/env nix-shell
#! nix-shell -i fish -p jq jaq bat

argparse f/force -- $argv; or exit 2

set -l reset (set_color normal)
set -l bold (set_color --bold)
set -l italics (set_color --italics)
set -l red (set_color red)
set -l green (set_color green)
set -l yellow (set_color yellow)
set -l blue (set_color blue)
set -l cyan (set_color cyan)
set -l magenta (set_color magenta)

set -l config_file config/comms_failure.json

if not test -f $config_file
    printf '%serror%s: %s does not exist!\n' $red $reset $config_file >&2
    exit 1
end

printf '%sinfo%s: starting communication failure experiments\n' $green $reset >&2

set -l t_start (date "+%s")

for seed in 0
    set -l json (jaq ".SEED = $seed" < $config_file)
    printf '%s\n' $json >$config_file
    printf '%sinfo%s: changed SEED to: %d\n' $green $reset $seed >&2

    for v0 in 10 15
        set -l json (jaq ".MAX_SPEED = $v0" < $config_file)
        printf '%s\n' $json >$config_file
        printf '%sinfo%s: changed target speed to: %d\n' $green $reset $v0 >&2

        seq 0.0 0.1 0.7 | string replace ',' '.' | while read probability
            set -l json (jaq ".COMMS_FAILURE_RATE = $probability" < $config_file)
            printf '%s\n' $json >$config_file
            bat $config_file

            printf '%sinfo%s: changed COMMS_FAILURE_RATE to: %s\n' $green $reset $probability >&2
            set -l output_file experiments/communications-failure-lm-3-tk-13.33/target-speed-$v0-probability-$probability-seed-$seed.json

            set -l t_end (date "+%s")
            set -l t_diff (math "$t_end - $t_start")
            if functions -q peopletime
                printf '%sinfo%s: time elapsed: \n' $green $reset (peopletime (math "$t_diff * 1000")) >&2
            end

            if test -f $output_file
                if not set -q _flag_force
                    printf '%swarn%s: %s already exists, use -f or --force to overwrite\n' $yellow $reset $output_file >&2
                    continue
                else
                    printf '%sinfo%s: overwriting %s\n' $green $reset $output_file >&2
                end
            end

            ./cmake-build-release/gbpplanner --cfg $config_file
            set -l exported_json metrics.json
            test -f $exported_json; or exit 1
            set -l dirname (path dirname "$output_file")
            command mkdir -p "$dirname"
            mv "$exported_json" "$output_file"

        end
    end
end
