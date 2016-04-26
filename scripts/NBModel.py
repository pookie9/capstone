class NBModel:
    """Learns a Naive Bayes model and gives predictions"""     

    #labels is a 2d array of the predicted labels, nodes neighbors are those diectly next to it, N/S and E/W
    def __init__(self,labels,smoothing=1):
        self.numClasses=max(max(labels))
        print self.numClasses
        self.counts=[smoothing]*(self.numClasses+1) for i in range(self.numClasses+1)]*(self.numClasses+1)#*(self.numClasses+1)]#how many times each class is connected to each of the other classes
        print self.counts
        for i in range(len(labels)):
            for j in range(len(labels[0])):
                if i>0:
                    self.counts[labels[i-1][j]][labels[i][j]]+=1
                if j>0:
                    self.counts[labels[i][j-1]][labels[i][j-1]]+=1
                if i<len(labels)-1:
                    print (labels[i+1][j],labels[i][j])
                    self.counts[labels[i+1][j]][labels[i][j]]+=1
                if j<len(labels[0])-1:
                    self.counts[labels[i][j+1]][labels[i][j-1]]+=1
        print self.counts


if __name__=='__main__':
    labels=[[1,2,2,1,2,2,1,1],
            [1,1,2,1,2,2,2,1]]
    NBModel(labels)
    
