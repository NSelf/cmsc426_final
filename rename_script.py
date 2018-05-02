import os

for filename in os.listdir("./Data/SingleObject/scene_001/frames"):
    if filename.startswith("image"):
        os.rename(filename, "frame" + filename[5:])
                
