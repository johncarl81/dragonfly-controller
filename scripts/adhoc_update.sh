#!/bin/bash

ansible-playbook -i inventory adhoc_update.yaml --ask-become-pass
