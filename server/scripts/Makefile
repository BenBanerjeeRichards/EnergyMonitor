.PHONY: test 
test:
	docker build . -t scripts && docker run --network="host" -e MINIO_ENDPOINT="localhost:9000" -e MINIO_ACCESS="PnUBgX8c5mmuHwdvitQh" -e MINIO_SECRET="2bJaUedTjUlRiEInr8ahhp3ZUEdMD7BOHobFNaZd" -e MINIO_BUCKET="energy"  -e MINIO_SECURE="false" scripts 

