import numpy as np
import cv2
import pandas as pd
from keras.models import Sequential
from keras.layers import Flatten , Dense , Lambda , Cropping2D , Activation
from keras.layers import Convolution2D , MaxPooling2D , Dropout , ELU , Conv2D
from keras.optimizers import Adam
from keras.layers.normalization import BatchNormalization
from keras.callbacks import ModelCheckpoint , EarlyStopping
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

# Batch Size for Training
batch_size = 128 
#Maximum number of epochs
nb_epochs = 20
augmented_images , augmented_measurements = [] , []

#Image Pre Processing  Utilities

# Function to resize the images to 160X60X3
def resize(image):
    import tensorflow as tf
    return tf.image.resize_images(image, [60, 160])


# Function to convert the image from RGB to HSV

def changeLight(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV) #convert it to hsv
    randomLight = 0.25 + np.random.rand() 
    hsv[:,:,2] =  hsv[:,:,2] * randomLight
    newImage = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    return newImage


def transform(image , measurement):
    rows , cols  = image.shape[0] , image.shape[1]
    transRange = 100
    numPixels = 10;
    valPixels  = 0.4
    transX = transRange * np.random.uniform() - transRange/2
    measurement  = measurement + transX/transRange *2 *valPixels
    transY = numPixels * np.random.uniform() - numPixels/2
    transMat = np.float32([[1,0,transX], [0,1,transY]])
    image = cv2.warpAffine(image, transMat, (cols, rows))
    return image


# flip the images horizonally to create more training data 
def augment(image , measurement):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    augmented_images.append(cv2.flip(image,1))
    augmented_measurements.append(-measurement)
    image = transform(image, measurement)
    if np.random.rand() <= 1.0:
        image = changeLight(image)
    augmented_images.append(image)
    augmented_measurements.append(measurement)


# Generator for the training model 
def generator(images , angles ,batch_size=128):
    num_train = len(images)
    while True:
        for offset in range(0,num_train, batch_size):
            X_batch = images[offset:offset+batch_size]
            y_batch = angles[offset:offset+batch_size]
            
            yield shuffle(X_batch , y_batch)


# Build and return Model
def get_model():
    model = Sequential()
    # Crop the images 70 Pixels on Y and 25 Pixels on X from Input size 160X320X3 
    # To remove unwanted information
    model.add(Cropping2D(cropping=((70,25), (0,0)),input_shape=(160,320,3)))

    # Resize the images
    model.add(Lambda(resize))
    
    #Normalize the  images
    model.add(Lambda(lambda x: (x/127.5) -1.0))

    model.add(Conv2D(3,(1,1) ,strides=(1,1) , padding='same',init= 'he_normal'))
    model.add(BatchNormalization())
    model.add(ELU())
    model.add(Conv2D(36,(5,5) ,strides=(4,4) , padding = 'same',init= 'he_normal'))
    model.add(BatchNormalization())
    model.add(ELU())
    model.add(Conv2D(48,(5,5) ,strides=(2,2) , padding ='same',init= 'he_normal'))
    model.add(BatchNormalization())
    model.add(ELU())

    model.add(Conv2D(64,(3,3), strides=(2,2) , padding ='same',init= 'he_normal'))

   # Fully Connected Layers , Dropouts are added to avoid overfiting also Batch Normalization acts as a regularizer
    model.add(Flatten())
    model.add(Dropout(0.2))
    model.add(BatchNormalization())
    model.add(ELU())

    model.add(Dense(1024))
    model.add(Dropout(0.5))
    model.add(BatchNormalization())
    model.add(ELU())

    model.add(Dense(512))
    model.add(Dropout(0.5))
    model.add(BatchNormalization())
    model.add(ELU())



    model.add(Dense(10))
    model.add(BatchNormalization())
    model.add(ELU())
    model.add(Dense(1))

    adam = Adam(lr=0.0001) # Learning rate = 0.0001

   #using Adam Optimizer and Loss Mean Square Error
    model.compile(loss='mse' , optimizer=adam, metrics = ['accuracy'])

    return model



def main():
    data = 'data_prg/data/'
    image_path  = data+'driving_log_linux.csv'
    dt = pd.DataFrame(data=pd.read_csv(image_path))

    center_images=[]
    #Loop the through the data frame to read only the center images
    for row in dt.values:
        center_source_path  = data+row[0]
        #print (center_source_path)
        center_image = cv2.imread(center_source_path, cv2.IMREAD_COLOR)
        steering = row[3]
        augment(center_image, steering)

   # We will   use only the center images for Validation
    augmented_images, X_val, augmented_measurements, y_val = train_test_split(augmented_images, augmented_measurements , test_size = 0.2, random_state=42)

    correction = 0.2 # Steering angle correction applied to left and right images , so it  looks like center images to the model
    # Read the left and right images
    for row in dt.values:
        left_source_path    = data+row[1].strip()
        right_source_path   = data+row[2].strip()
        #print(left_source_path)
        steering = row[3]
        left_image = cv2.imread(left_source_path, cv2.IMREAD_COLOR)
        right_image = cv2.imread(right_source_path, cv2.IMREAD_COLOR)
        left_steering = (steering + correction) # correction added to left
        right_steering = (steering -correction) # correction substracted from right
        augment(left_image ,left_steering)
        augment(right_image, right_steering)

    #Convert the training  images to numpy array 
    X_samples = np.array(augmented_images)
    y_samples = np.array(augmented_measurements)

    #Convert the validation images to numpy array
    X_val = np.array(X_val)
    y_val = np.array(y_val)

    model = get_model()
    print ("Model Summary :\n" , model.summary())

    # create the  training  generator and validation generator 
    train_generator = generator(X_samples, y_samples, batch_size)
    validation_generator = generator(X_val , y_val , batch_size)

    # Create Model CheckPoint call backs to store the best weights
    checkpointer = ModelCheckpoint(filepath = "./v2-weights.{epoch:02d}-{val_loss:.2f}.hdf5" , verbose =1 , save_best_only = True)

   # Stop running epochs if the validation loss doesnt improve afer 3 epoch.
    early_stopping = EarlyStopping(monitor='val_loss', patience=3)

    # Fit/train the model , using fit_generator as we are using generators for training and validation data. 
    model.fit_generator(train_generator,
                    steps_per_epoch= len(X_samples)/batch_size,
                    validation_data= validation_generator,
                    validation_steps = len(X_val)/batch_size,
                    epochs = nb_epochs,
                    callbacks = [checkpointer,early_stopping])
   
   # Save the model to the  disk
    model.save("model.h5")
    print("Model Saved to the disk")


if __name__ == '__main__':
    main()