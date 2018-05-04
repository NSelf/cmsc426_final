import os
path = "./Data/SingleObject/scene_001/frames/"
os.chdir(path)
for filename in os.listdir("."):
    if filename.startswith("image"):
        os.rename(filename, "frame" + filename[5:])
        #print(filename)
