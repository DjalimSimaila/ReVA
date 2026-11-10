ffmpeg -i video.mp4 -vf "fps=1" -q:v 2 images/frame_%03d.jpg
