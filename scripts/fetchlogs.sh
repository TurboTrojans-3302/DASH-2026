#!/bin/sh
DEST=~/Documents/3302/logfiles
REMOTE_DIRS='/home/lvuser/logs /u/logs'

while true; do 
    hostlist='10.33.2.2 172.22.11.2 roborio-3302-FRC.local'
    for host in $hostlist; do
        echo "Trying $host..."

        # Get list of files on remote as "size basename fullpath"
        remote_files=$(sshpass -p "" ssh \
            -o StrictHostKeyChecking=no \
            -o ConnectTimeout=5 \
            lvuser@$host \
            "find $REMOTE_DIRS -type f -name '*.wpilog' 2>/dev/null \
             | xargs -I{} sh -c 'stat -c \"%s %n\" \"{}\"'") || continue

        # Get list of local files as "size basename"
        local_files=$(find "$DEST" -type f -name '*.wpilog' \
            | xargs -I{} sh -c 'stat -f "%z %N" "{}"' 2>/dev/null \
            | awk '{size=$1; n=split($2,a,"/"); print size, a[n]}')

        # Build sftp batch: fetch files where name+size don't match a local file
        batch=""
        echo "$remote_files" | while IFS= read -r line; do
            [ -z "$line" ] && continue
            rsize=$(echo "$line" | awk '{print $1}')
            rpath=$(echo "$line" | awk '{print $2}')
            rname=$(basename "$rpath")

            # Check if local has a file with same name AND same size
            if ! echo "$local_files" | awk -v s="$rsize" -v n="$rname" '$1==s && $2==n {found=1} END {exit !found}'; then
                batch="$batch
-get $rpath $DEST/$rname"
            fi
        done

        if [ -z "$batch" ]; then
            echo "No new files from $host"
            continue
        fi

        echo "$batch" | sshpass -p "" sftp \
            -o StrictHostKeyChecking=no \
            -o ConnectTimeout=5 \
            lvuser@$host
    done
    sleep 60 
done