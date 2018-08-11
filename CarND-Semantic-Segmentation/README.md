# Semantic Segmentation
### Introduction
This project goal is to  label the pixels of a road in images using a Fully Convolutional Network (FCN). Semantic segmenation is the ability predict every pixels of a image moving corase to fine inference.

#### Prerequities 
 - [Python 3](https://www.python.org/)
 - [TensorFlow](https://www.tensorflow.org/)
 - [NumPy](http://www.numpy.org/)
 - [SciPy](https://www.scipy.org/)
 
 ##### Dataset
Download the [Kitti Road dataset](http://www.cvlibs.net/datasets/kitti/eval_road.php) from [here](http://www.cvlibs.net/download.php?file=data_road.zip).  Extract the dataset in the `data` folder.  This will create the folder `data_road` with all the training a test images.

### Setup
##### GPU
`main.py` will check to make sure you are using GPU - if you don't have a GPU on your system, you can use AWS or another cloud computing platform.  I used a AWS spot  instance to train the model using tensorflow 1.9. I used the base AWS Deep Learning Ubuntu AIM , had to upgrade tensorflow to 1.9 and install tqdm and pillow .
I initally build the model locally and test it for compilation and correctness before running it on the AWS instance this way we dont end wasting time on the AWS instance to debug .

### Start
##### Implementation
I used the project walkthrough video as  starter to implementation, 
Downloading the pre trained VGG model is  implemented in the fucntion load_VGG  code lines  11 to 37.
A Fully convolutions networks layers are build using a 1X1 convolution of the inputs layers  and then applying the deconvolutions also skip layers are added.
For the convolutions a random initializer with stdev of 0.01 and L2 regularization of 1e-03 is used  code line 41 to 79.
Cross entropy loss function with a Learning rate of 1e-04 is used and Adam optimizer to minimze the loss.
Model train is implemenyed in code 102 to 133.
Finally the model is trained with batch size of 1 for 10 epochs . I experimented with different batch size of 2 , 4 , 8 etc but every time the AWS instance will fail due to GPU running out of memeory. 
For the final 10th epoch the average loss is  0.05 with stdev of 0.042.

Predicted images are available in the  directory runs/1533892587.3156998.

![Sample Predictions] [https://github.com/Praveenraj49/Udacity/blob/master/CarND-Semantic-Segmentation/runs/1533892587.3156998/uu_000085.png]

![Sample Predictions] [https://github.com/Praveenraj49/Udacity/blob/master/CarND-Semantic-Segmentation/runs/1533892587.3156998/um_000025.png]

![Sample Predictions] [https://github.com/Praveenraj49/Udacity/blob/master/CarND-Semantic-Segmentation/runs/1533892587.3156998/um_000008.png]

![Sample Predictions] [https://github.com/Praveenraj49/Udacity/blob/master/CarND-Semantic-Segmentation/runs/1533892587.3156998/um_0000031.png]




##### Run
Run the following command to run the project:
```
python main.py
```
