# sky 360 Cyclop

This is the fisheye robotic part for the `qhy183` camera.

* note: `sky360/cyclop` image is not published yet.

## requirements
- docker 

## run amd64 
```bash
docker run --privileged -it sky360/cyclop:latest
```
## run arm64
```bash
docker run --privileged -it --platform linux/arm64 sky360/cyclop:latest
```

## For development

### build
```bash
make build
```
### run 
```bash
make run
```


