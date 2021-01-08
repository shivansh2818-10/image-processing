#!/usr/bin/env bash
set -e
INSTALL_DIR=${1:-$PWD/bats}

if [ ! -d "$INSTALL_DIR" ]
then
    echo "Installing bats to $INSTALL_DIR"
	TMP_DIR=$(mktemp -u)
	git clone https://github.com/sstephenson/bats.git $TMP_DIR
	$TMP_DIR/install.sh $INSTALL_DIR
	rm -rf $TMP_DIR
fi
