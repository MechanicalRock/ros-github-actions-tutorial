setup:
	docker rm --force ros-github-actions
	docker run -t -d --name ros-github-actions --user=root -v ${PWD}:/root/ ros:melodic-robot

start:
	docker start ros-github-actions

shell:
	docker exec -it -w /root ros-github-actions bash

stop:
	docker stop ros-github-actions