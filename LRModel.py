from sklearn import linear_model, metrics;
import numpy;
import cv2;

class LRModel:
    """Learns a Logistic Regression model using the pixels as attributes"""

    #img is an opencv image, note, I think we want to remove the robot from this image by stitching together two images...
    def __init__(self, img,labels,ridge=None):
        self.img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.img=img
        (self.height,self.width,j)=self.img.shape
        self.attrs=numpy.array(self.img).flatten().reshape(img.shape[0]*img.shape[1],3) #Reshaping it into a numpixels by 3 (HSV) array
        self.ridge=ridge
        #Note that the regularization is stronger for a smaller C, so I inverse the ridge
        if ridge==None:
            self.classifier=linear_model.LogisticRegression()
        else:
            self.classifier=linear_model.LogisticRegression(C=ridge)
        labels=labels.flatten()
        self.knownIndices=numpy.where(labels>0)
        self.knownLabels=labels[self.knownIndices]
        self.unknownIndices=numpy.where(labels==0)

        self.classifier.fit(self.attrs[self.knownIndices],self.knownLabels)
        
    def predict(self, twoD=False):
        labels=self.classifier.predict(self.attrs)
        labels[self.knownIndices]=self.knownLabels
        if twoD:
            labels.resize((self.height,self.width))
        return labels
    def predictProbs(self):
        probs=self.classifier.predict_proba(self.attrs)
        return probs

    def predictWithMetrics(self,trueLabels):
        trueLabels=trueLabels.flatten()
        predLabels=self.predict()
        testLabels=trueLabels[self.unknownIndices]
        testPred=predLabels[self.unknownIndices]
        print "Logistic regression using raw pixel features:\n%s\n" % (metrics.classification_report(testLabels, testPred))

    def predictAndShow(self,trueLabels=None):
        preds=self.predict()
        preds=numpy.resize(preds,(self.height,self.width))
        predDriveable=numpy.where(preds==2)
        predPic=numpy.zeros((self.height,self.width,3))
        predPic[predDriveable]=(255,255,255)
        cv2.imshow("Predicted",predPic)

        if trueLabels!=None:
            trueDriveable=numpy.where(trueLabels==2)
            truePic=numpy.zeros((self.height,self.width,3))
            truePic[trueDriveable]=(255,255,255)
            cv2.imshow("True",truePic)


        cv2.waitKey(0)
        cv2.destroyAllWindows()
