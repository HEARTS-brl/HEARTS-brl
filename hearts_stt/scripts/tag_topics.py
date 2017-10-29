

class tag_topics():
# Author : Derek Ripper
# Created: 19 June 2017
# Purpose: Problem: when Pubhishing and subscribing data in ROS there is no
#          guarantee that the subscribers will be in sync with the publisers.
#
#          Hence this code falcilates prefixing each topic with a unique 4
#          digit code in the form of #xxxx# where xxxx is an integer. xxxx
#          is teh sequential file number and is also used as the row index
#          to store results.
#
#          When a subsciber picks up a topic via a "callbak" then the same
#          prefix will have been added so that the result can be stored against
#          the correct requester.

    def bld_arr(self,irows):
        # irows :number of rows of data (Base Zero)
        self.irows = irows + 1 # add a row for EXCEL headers
        
        # 7 columns 
        self.icols=6

        # intialise a 2-D array of j columns x i rows
        self.myArray=[[0 for j in range(self.icols)] for i in range(self.irows)]

        # use row zero for EXCEL headers
        self.myArray[0]=['Audio file name',
                    'True TEXT',
                    'Derived TEXT',
                    'TEXT Match?',
                    'True  CFR',
                    'Derived CFR',
                    'CFR Match?' ]
        
        default_row = ["Undefined"] * 5
        for i in range(1,irows):
            self.myArray[i]= default_row  
        return 

    def add_key(self, ikey, topic_string):
        # Only used when Publishing Topics
        # ikey   : row index in ground_truth.csv file
        # msg    : ROS Topic string
        self.isKey ='#'+str(ikey).zfill(4)+'#'
        return self.isKey+topic_string


    def get_key(self,msg):
        # msg : ROS Topic string with #xxxx# prefix
        # return xxxx string as Integer Index
        row_index = int(msg[1:5])
        
        # return clean clean ROS Topic string
        topic_string  = msg[6:]
        
        return row_index, topic_string
      
    def store_inputs(self, ikey, Wavfile,TrueText,TrueCFR):
         # ikey   : row index in ground_truth.csv file
         #
         # comparison formulae for S/Sheet (+1 as sheet has a hearder row)
         ssrow = ikey+1
         comptext = '=B'+str(ssrow)+'=C'+str(ssrow)
         compcfr  = '=E'+str(ssrow)+'=F'+str(ssrow)

         myList = [Wavfile,TrueText,"Not defined",comptext,TrueCFR,"Not Defined",compcfr]
         self.myArray[ikey] = myList
         return
        
    def store_result(self, iTopic, topic_msg):
         # iTopic : col index key 2 = Speech to text result
         #                        5 = Text    to CFR result
         
         row_index, topic_string = self.get_key(topic_msg) 
         
         self.myArray[row_index][iTopic] = topic_string
         return

    def list_results(self):
        print(self.myArray)

    def write_results_to_file(self, filename):
        fh = open(filename,'w')
        sep = '|'
        for i in range(0, self.irows):
            row = self.myArray[i][0]+sep+ \
                  self.myArray[i][1]+sep+ \
                  self.myArray[i][2]+sep+ \
                  self.myArray[i][3]+sep+ \
                  self.myArray[i][4]+sep+ \
                  self.myArray[i][5]+sep+ \
                  self.myArray[i][6]+  '\n'
            fh.write(row)
        fh.close()   
        
#################################################################################
        

