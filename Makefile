SRC_DIR=src
SCRIPTS_DIR=shell_scripts

.PHONY: all
all:
			$(SCRIPTS_DIR)/run.sh

.PHONY: clean
clean:
			rm -f $(SRC_DIR)/*.pyc
			rm -rf $(SRC_DIR)/__pycache__/ 
