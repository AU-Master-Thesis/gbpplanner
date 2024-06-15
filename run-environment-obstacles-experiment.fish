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

set -l config_file config/environment-obstacles.json

if not test -f $config_file
    printf '%serror%s: %s does not exist!\n' $red $reset $config_file >&2
    exit 1
end

set -l t_start (date "+%s")

for seed in 0
    set -l json (jaq ".SEED = $seed" < $config_file)
    printf '%s\n' $json >$config_file
    printf '%sinfo%s: changed SEED to: %d\n' $green $reset $seed >&2

    for n in (seq 5 5 50)
        set -l json (jaq ".NUM_ROBOTS = $n" < $config_file)
        printf '%s\n' $json >$config_file
        printf '%sinfo%s: changed #robots to: %d\n' $green $reset $n >&2

        set -l t_end (date "+%s")
        set -l t_diff (math "$t_end - $t_start")
        if functions -q peopletime
            printf '%sinfo%s: time elapsed: %s\n' $green $reset (peopletime (math "$t_diff * 1000")) >&2
        end

        set -l output_file experiments/environment-obstacles-lm-3-tk-5-mi-25-me-25/num-robots-$n-seed-$seed.json

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
