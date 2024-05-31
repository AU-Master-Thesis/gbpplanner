#!/usr/bin/env nix-shell
#! nix-shell -i fish -p jq jaq

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


set -l config_file config/varying_network.json

if not test -f $config_file
    printf '%serror%s: %s does not exist!\n' $red $reset $config_file >&2
    exit 1
end

printf '%sinfo%s: starting experiment\n' $green $reset >&2

set -l t_start (date "+%s")

for seed in 0
    set -l json (jaq ".SEED = $seed" < $config_file)
    printf '%s\n' $json >$config_file
    printf '%sinfo%s: changed SEED to: %d\n' $green $reset $seed >&2

    for comms_radius in 20 40 60 80
        set -l json (jaq ".COMMUNICATION_RADIUS = $comms_radius" < $config_file)
        printf '%s\n' $json >$config_file

        printf '%sinfo%s: changed COMMUNICATION_RADIUS to: %d\n' $green $reset $comms_radius >&2
        set -l output_file experiments/varying-network-connectivity/comms-radius-$comms_radius-seed-$seed.json

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
