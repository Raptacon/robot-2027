.PHONY: sim

CWD=${CURDIR}

ifeq ($(OS), Windows_NT)
VENV=.venv_windows
PYTHON=python
VENVBIN=./${VENV}/Scripts
else ifneq ("$(wildcard /.dockerenv)","")
VENV=.venv_docker
PYTHON=python3
VENVBIN=./${VENV}/bin
else
VENV=.venv_osx
PYTHON=python3
VENVBIN=./${VENV}/bin
endif

default: help 
# https://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
help: ## This list of Makefile targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

sim: setup_${VENV} ## Performs coverage tests and then runs the simulator
	${VENVBIN}/${PYTHON} -m robotpy coverage sim

run: ## Runs the robot
	${VENVBIN}/${PYTHON} -m robotpy run

${VENV}:
	${PYTHON} -m venv ${VENV}

lint: ## Runs the linter(s)
	${VENVBIN}/flake8 .

test: setup_${VENV} lint  coverage ## Does a lint and then test
	${VENVBIN}/${PYTHON} -m robotpy test

coverage: setup_${VENV} test
	${VENVBIN}/${PYTHON} -m robotpy coverage

setup_${VENV}: ${VENV}
	${VENVBIN}/${PYTHON} -m pip install --upgrade pip setuptools
	${VENVBIN}/pip install --pre -r ${CWD}/requirements.txt
	$(file > setup_${VENV})

clean:
	rm -f setup setup_${VENV}

realclean: clean
	rm -fr ${VENV}

docker: docker_build
	docker run --rm -ti -v $$(PWD):/src raptacon2027_build bash

docker_build:
	docker build . --tag raptacon2027_build

# Installs the 3rd party dependencies such as photonvision and rapatcon3200 (whatever is in the toml esp in the requires section)
# https://docs.wpilib.org/en/stable/docs/software/python/pyproject_toml.html
sync:
	${PYTHON} -m robotpy sync

deploy: sync
	${PYTHON} -m robotpy deploy
