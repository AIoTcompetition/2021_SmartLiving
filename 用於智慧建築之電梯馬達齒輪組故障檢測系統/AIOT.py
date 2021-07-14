from pynq import Overlay
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pynq import Xlnk
from pynq.lib.video import *
from scipy.signal import hilbert
from scipy.fftpack import fft
import smtplib
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.image import MIMEImage
from email.header import Header
from email import encoders

from header import plotpath
smtpHost = "smtp.gmail.com"
smtpPort = 587

#SMTP Connection Account Settings
smtpUserName = 'aiotgear@gmail.com'
smtpUserPassword = 'ceres5037'

#Email Account Setting
senderEmail = 'justintang@cereal.cse.nsysu.edu.tw'
senderEmailPassword = 'Asdzxc189'
receiverEmail = ['aa3511tw@gmail.com','justintang@cereal.cse.nsysu.edu.tw']

img_file = open(r'/home/xilinx/jupyter_notebooks/fig.JPG','rb')
img_data = img_file.read()
img_file.close()
img = MIMEImage(img_data)
img.add_header('Content-ID','dns_config')


import time
localtime = time.localtime()
result = time.strftime("%Y-%m-%d/%I:%M:%S", localtime)
#result ="2021-07-08 / 17:26:47"
start = time.time()

label = ['in_duan', 'in_mo', 'in_normal',
         'out_duan', 'out_mo', 'out_normal',
         'normal_duan', 'normal_mo', 'normal_normal']

class MXIC():
    def __init__(self):
        self.name = 'MXIC'
        self.data_in = None
        self.data_out = None
        self.head = 0
        self.ans = [0] * 10
        self.send = 0  

    def LOAD_DATA(self):
        global offset
        f = pd.read_csv('out.csv',usecols=[4],nrows=512)
        m_wave_data = f
        m_wave_data = np.array(m_wave_data)
        #m_wave_data = m_wave_data[150:150 + 256]
        #m_wave_data = abs(fft(m_wave_data))
        m_wave_data = m_wave_data[24:50]
        self.data_in = m_wave_data 
        print('Dataload finish!!')
        
    #def LOAD_DATA(self):
        #f = np.load('input_vibration.npy')
        #m_wave_data = f
        #m_wave_data = m_wave_data[150:150 + 256]
        #m_wave_data = abs(fft(m_wave_data))
        #m_wave_data = m_wave_data[5:30]

        #self.data_in = m_wave_data / 65536 / 2 * 1024 * 1024

    def Predict(self):
        print('Start Predict.......')
        self.dma = spi.axi_dma_0
     
        xlnk = Xlnk()
        
        dma_in = xlnk.cma_array(shape=(25,), dtype=np.uint32)
       
        dma_out = xlnk.cma_array(shape=(25,), dtype=np.uint32)
      
        
        for i in range(25):
            dma_in[i] = int(self.data_in[i])
    
        self.dma.sendchannel.transfer(dma_in)
       
        self.dma.recvchannel.transfer(dma_out)
        
        self.dma.sendchannel.wait()
        
        self.dma.recvchannel.wait()
        
        self.data_out = dma_out

    def Result(self):
        ans = self.data_out[:4]
        p = np.argmax(ans)
        self.ans[p]+=1

    def Plot(self):
        
        gear = ''
        p = np.argmax(self.ans[:4])
        #p =5
        if p == 0 and self.send ==0 :
            self.send = 1
            gear = ' Broken '
            mail_msg ='''
        </br>
        <div style="color:rgb(0, 140, 255);font-size:35px;">Gear fault alert</div>
        </br>
        <p style="font-size:15px;">
            We think that the Gear of your equipment has been worn out.  When this happen,<br/>
            we require you to replace the Gear as soon as possible, or the abnormal vibration<br/>
            will affect other parts of equipment.
        </p><br/>
        <p>
            <b>Alert Time</b>:'''+result+'''   <br/>
            <b>Gear Fault Type</b>: Wear<br/>
            <b>Decription</b>:<br/>
            <b>Long-term operation causes the gear wear, please repair it as soon as possible to avoid more serious damage<br/>
        </p>
        <p style="font-size:15px;">For more detial and history records, click the button below.
        <br/>
         Go to google drive: <a href="https://drive.google.com/drive/folders/1TfqL4hkC3Yr3UZK3unoKDB7Rybyn6Hpy?usp=sharing">click here</a>
        <p><b style="color:rgb(10, 107, 187)">Note: </b>Please do not ignore this email to make your equipment worse<br/>
        Thanks,<br/>
         <p>The attached file is the time domain and frequency domain waveforms of the original signal.<br/>
        
'''
            # define deliver info
            message = MIMEMultipart()
            message['From'] = Header('PYNQ-Z2', "utf-8")
            message['To'] = Header('The Personnel', "utf-8")
            subject = "Gear Fault Alert"
            message['Subject'] = Header(subject, "utf-8")
            #smtpObj = smtplib.SMTP( [host [, post [, local_hostname]]])

            # message content
            msg_content = MIMEText(mail_msg, 'html', 'utf-8')

            # attach the message to email
            message.attach(msg_content)
            message.attach(img)
            

            #with open(plotpath + '/merge.png', 'rb') as img:
                #msg_img = MIMEBase('img1', 'png', filename = 'merge.png')
                #msg_img.add_header('Content-Disposition', 'attachment', filename='merge.png')
                #msg_img.add_header('X-Attachment-Id', '0')
                #msg_img.add_header('Content-ID', '<0>')
                #msg_img.set_payload(img.read())
                #encoders.encode_base64(msg_img)
                #message.attach(msg_img)



            try:
                #smtpObj = smtplib.SMTP(smtpHost, smtpPort)
                smtpObj = smtplib.SMTP()
                print("SMTP Init")
                smtpObj.connect(smtpHost, smtpPort)
                print("SMTP Connect")
                smtpObj.ehlo()
                smtpObj.starttls()
                smtpObj.ehlo()
                print("SMTP ehlo")
                smtpObj.login(smtpUserName, smtpUserPassword)
                print("SMTP Login")
                smtpObj.sendmail(senderEmail, receiverEmail, message.as_string())
                print("Email Sent")
            except smtplib.SMTPException as error:
                print(str(error))
                print("Error: Cannot Send Email")

        elif p == 1 and slef.send == 0:
            slef.send = 1
            
            gear = '  Wear  '
            mail_msg = '''
         </br>
        <div style="color:rgb(0, 140, 255);font-size:35px;">Gear fault alert</div>
        </br>
        <p style="font-size:15px;">
            We think that the Gear of your equipment has been worn out.  When this happen,<br/>
            we require you to replace the Gear as soon as possible, or the abnormal vibration<br/>
            will affect other parts of equipment.
        </p><br/>
        <p>
            <b>Alert Time</b>:'''+result+'''   <br/>
            <b>Gear Fault Type</b>: Wear<br/>
            <b>Decription</b>:<br/>
            <b>Long-term operation causes the gear wear, please repair it as soon as possible to avoid more serious damage<br/>
        </p>
        <p style="font-size:15px;">For more detial and history records, click the button below.
        <br/>
         Go to google drive: <a href="https://drive.google.com/drive/folders/1TfqL4hkC3Yr3UZK3unoKDB7Rybyn6Hpy?usp=sharing">click here</a>
        <p><b style="color:rgb(10, 107, 187)">Note: </b>Please do not ignore this email to make your equipment worse<br/>
        Thanks,<br/>
         <p>The attached file is the time domain and frequency domain waveforms of the original signal.<br/>
'''
            # define deliver info
            message = MIMEMultipart()
            message['From'] = Header('PYNQ-Z2', "utf-8")
            message['To'] = Header('The Personnel', "utf-8")
            subject = "Gear Fault Alert"
            message['Subject'] = Header(subject, "utf-8")
            #smtpObj = smtplib.SMTP( [host [, post [, local_hostname]]])

            # message content
            msg_content = MIMEText(mail_msg, 'html', 'utf-8')

            # attach the message to email
            message.attach(msg_content)


            #with open(plotpath + '/merge.png', 'rb') as img:
                #msg_img = MIMEBase('img1', 'png', filename = 'merge.png')
                #msg_img.add_header('Content-Disposition', 'attachment', filename='merge.png')
                #msg_img.add_header('X-Attachment-Id', '0')
                #msg_img.add_header('Content-ID', '<0>')
                #msg_img.set_payload(img.read())
                #encoders.encode_base64(msg_img)
                #message.attach(msg_img)



            try:
                #smtpObj = smtplib.SMTP(smtpHost, smtpPort)
                smtpObj = smtplib.SMTP()
                print("SMTP Init")
                smtpObj.connect(smtpHost, smtpPort)
                print("SMTP Connect")
                smtpObj.ehlo()
                smtpObj.starttls()
                smtpObj.ehlo()
                print("SMTP ehlo")
                smtpObj.login(smtpUserName, smtpUserPassword)
                print("SMTP Login")
                smtpObj.sendmail(senderEmail, receiverEmail, message.as_string())
                print("Email Sent")
            except smtplib.SMTPException as error:
                print(str(error))
                print("Error: Cannot Send Email")
        elif p == 2 and slef.send == 0:
            slef.send = 1
            gear = 'Misalignment '
            mail_msg = '''
         </br>
        <div style="color:rgb(0, 140, 255);font-size:35px;">Gear fault alert</div>
        </br>
        <p style="font-size:15px;">
            We think that the Gear of your equipment has been worn out.  When this happen,<br/>
            we require you to replace the Gear as soon as possible, or the abnormal vibration<br/>
            will affect other parts of equipment.
        </p><br/>
        <p>
            <b>Alert Time</b>:'''+result+'''   <br/>
            <b>Gear Fault Type</b>: Wear<br/>
            <b>Decription</b>:<br/>
            <b>Long-term operation causes the gear wear, please repair it as soon as possible to avoid more serious damage<br/>
        </p>
        <p style="font-size:15px;">For more detial and history records, click the button below.
        <br/>
         Go to google drive: <a href="https://drive.google.com/drive/folders/1TfqL4hkC3Yr3UZK3unoKDB7Rybyn6Hpy?usp=sharing">click here</a>
        <p><b style="color:rgb(10, 107, 187)">Note: </b>Please do not ignore this email to make your equipment worse<br/>
        Thanks,<br/>
         <p>The attached file is the time domain and frequency domain waveforms of the original signal.<br/>
'''
            # define deliver info
            message = MIMEMultipart()
            message['From'] = Header('PYNQ-Z2', "utf-8")
            message['To'] = Header('The Personnel', "utf-8")
            subject = "Gear Fault Alert"
            message['Subject'] = Header(subject, "utf-8")
            #smtpObj = smtplib.SMTP( [host [, post [, local_hostname]]])

            # message content
            msg_content = MIMEText(mail_msg, 'html', 'utf-8')

            # attach the message to email
            message.attach(msg_content)


            #with open(plotpath + '/merge.png', 'rb') as img:
                #msg_img = MIMEBase('img1', 'png', filename = 'merge.png')
                #msg_img.add_header('Content-Disposition', 'attachment', filename='merge.png')
                #msg_img.add_header('X-Attachment-Id', '0')
                #msg_img.add_header('Content-ID', '<0>')
                #msg_img.set_payload(img.read())
                #encoders.encode_base64(msg_img)
                #message.attach(msg_img)



            try:
                #smtpObj = smtplib.SMTP(smtpHost, smtpPort)
                smtpObj = smtplib.SMTP()
                print("SMTP Init")
                smtpObj.connect(smtpHost, smtpPort)
                print("SMTP Connect")
                smtpObj.ehlo()
                smtpObj.starttls()
                smtpObj.ehlo()
                print("SMTP ehlo")
                smtpObj.login(smtpUserName, smtpUserPassword)
                print("SMTP Login")
                smtpObj.sendmail(senderEmail, receiverEmail, message.as_string())
                print("Email Sent")
            except smtplib.SMTPException as error:
                print(str(error))
                print("Error: Cannot Send Email")
        elif p == 3 and slef.send == 0:
            slef.send =1
            gear = ' eccentric '
            mail_msg = '''
         </br>
        <div style="color:rgb(0, 140, 255);font-size:35px;">Gear fault alert</div>
        </br>
        <p style="font-size:15px;">
            We think that the Gear of your equipment has been worn out.  When this happen,<br/>
            we require you to replace the Gear as soon as possible, or the abnormal vibration<br/>
            will affect other parts of equipment.
        </p><br/>
        <p>
            <b>Alert Time</b>:'''+result+'''   <br/>
            <b>Gear Fault Type</b>: Wear<br/>
            <b>Decription</b>:<br/>
            <b>Long-term operation causes the gear wear, please repair it as soon as possible to avoid more serious damage<br/>
        </p>
        <p style="font-size:15px;">For more detial and history records, click the button below.
        <br/>
         Go to google drive: <a href="https://drive.google.com/drive/folders/1TfqL4hkC3Yr3UZK3unoKDB7Rybyn6Hpy?usp=sharing">click here</a>
        <p><b style="color:rgb(10, 107, 187)">Note: </b>Please do not ignore this email to make your equipment worse<br/>
        Thanks,<br/>
         <p>The attached file is the time domain and frequency domain waveforms of the original signal.<br/>
'''
            # define deliver info
            message = MIMEMultipart()
            message['From'] = Header('PYNQ-Z2', "utf-8")
            message['To'] = Header('The Personnel', "utf-8")
            subject = "Gear Fault Alert"
            message['Subject'] = Header(subject, "utf-8")
            #smtpObj = smtplib.SMTP( [host [, post [, local_hostname]]])

            # message content
            msg_content = MIMEText(mail_msg, 'html', 'utf-8')

            # attach the message to email
            message.attach(msg_content)


            #with open(plotpath + '/merge.png', 'rb') as img:
                #msg_img = MIMEBase('img1', 'png', filename = 'merge.png')
                #msg_img.add_header('Content-Disposition', 'attachment', filename='merge.png')
                #msg_img.add_header('X-Attachment-Id', '0')
                #msg_img.add_header('Content-ID', '<0>')
                #msg_img.set_payload(img.read())
                #encoders.encode_base64(msg_img)
                #message.attach(msg_img)



            try:
                #smtpObj = smtplib.SMTP(smtpHost, smtpPort)
                smtpObj = smtplib.SMTP()
                print("SMTP Init")
                smtpObj.connect(smtpHost, smtpPort)
                print("SMTP Connect")
                smtpObj.ehlo()
                smtpObj.starttls()
                smtpObj.ehlo()
                print("SMTP ehlo")
                smtpObj.login(smtpUserName, smtpUserPassword)
                print("SMTP Login")
                smtpObj.sendmail(senderEmail, receiverEmail, message.as_string())
                print("Email Sent")
            except smtplib.SMTPException as error:
                print(str(error))
                print("Error: Cannot Send Email")
        elif p == 4 and slef.send == 0:
            slef.send = 1
            gear = '  Normal  ' 
            mail_msg = '''
         </br>
        <div style="color:rgb(0, 140, 255);font-size:35px;">Gear fault alert</div>
        </br>
        <p style="font-size:15px;">
            We think that the Gear of your equipment has been worn out.  When this happen,<br/>
            we require you to replace the Gear as soon as possible, or the abnormal vibration<br/>
            will affect other parts of equipment.
        </p><br/>
        <p>
            <b>Alert Time</b>:'''+result+'''   <br/>
            <b>Gear Fault Type</b>: Wear<br/>
            <b>Decription</b>:<br/>
            <b>Long-term operation causes the gear wear, please repair it as soon as possible to avoid more serious damage<br/>
        </p>
        <p style="font-size:15px;">For more detial and history records, click the button below.
        <br/>
         Go to google drive: <a href="https://drive.google.com/drive/folders/1TfqL4hkC3Yr3UZK3unoKDB7Rybyn6Hpy?usp=sharing">click here</a>
        <p><b style="color:rgb(10, 107, 187)">Note: </b>Please do not ignore this email to make your equipment worse<br/>
        Thanks,<br/>
         <p>The attached file is the time domain and frequency domain waveforms of the original signal.<br/>
'''
            # define deliver info
            message = MIMEMultipart()
            message['From'] = Header('PYNQ-Z2', "utf-8")
            message['To'] = Header('The Personnel', "utf-8")
            subject = "Gear Fault Alert"
            message['Subject'] = Header(subject, "utf-8")
            #smtpObj = smtplib.SMTP( [host [, post [, local_hostname]]])

            # message content
            msg_content = MIMEText(mail_msg, 'html', 'utf-8')

            # attach the message to email
            message.attach(msg_content)


            #with open(plotpath + '/merge.png', 'rb') as img:
                #msg_img = MIMEBase('img1', 'png', filename = 'merge.png')
                #msg_img.add_header('Content-Disposition', 'attachment', filename='merge.png')
                #msg_img.add_header('X-Attachment-Id', '0')
                #msg_img.add_header('Content-ID', '<0>')
                #msg_img.set_payload(img.read())
                #encoders.encode_base64(msg_img)
                #message.attach(msg_img)



            try:
                #smtpObj = smtplib.SMTP(smtpHost, smtpPort)
                smtpObj = smtplib.SMTP()
                print("SMTP Init")
                smtpObj.connect(smtpHost, smtpPort)
                print("SMTP Connect")
                smtpObj.ehlo()
                smtpObj.starttls()
                smtpObj.ehlo()
                print("SMTP ehlo")
                smtpObj.login(smtpUserName, smtpUserPassword)
                print("SMTP Login")
                smtpObj.sendmail(senderEmail, receiverEmail, message.as_string())
                print("Email Sent")
            except smtplib.SMTPException as error:
                print(str(error))
                print("Error: Cannot Send Email")
            
            
        else :
            gear = 'normal'

        print('------------------------------------------\n')
        print('--  Index    Gear  -----\n')
        print('--   {}  --{} -----\n'.format(p,gear))
        print('------------------------------------------\n')
        

m_MXIC = MXIC()


spi = Overlay("/home/xilinx/jupyter_notebooks/RF_TEST_25_3.bit")





#m_MXIC.Collect_data()
count = 0

while True:
    m_MXIC.LOAD_DATA()
    m_MXIC.Predict()
    m_MXIC.Result()
    
    count += 1
    if count == 100 :
        m_MXIC.send = 0
    

    end = time.time()

    print("run time: %f s" % (end - start))
    m_MXIC.Plot()
    time.sleep(10)
