# -*- coding: utf-8 -*-

import threading
import time
import mraa
from firebase import firebase


class Secagem():
    def __init__(self, pwm):
        self.fila_leitura = [1, 1]
        self.pwm_value = pwm 
        self.firebase = firebase.FirebaseApplication('https://sd-project20172.firebaseio.com/status', None)
        self.data = {"Temperatura": 0, "Luminosidade": 0, "Time": 0, "CurvaSecagem": 0, "original": 0}        
        self.firebase.delete('/status', '')
        self.time_teste = 0
        self.timer_ref = time.time()
        self.curva_original = 0


    def ler_sensores(self):
        sensor_ldr = mraa.Aio(0)
        sensor_ntc = mraa.Aio(1)
       
        pwm_temp = mraa.Pwm(5)
        pwm_temp.period_us(350)
        pwm_temp.enable(True)

        pwm_lum = mraa.Pwm(6)
        pwm_lum.period_us(350)
        pwm_lum.enable(True)


        # print("Portas analogicas setadas")
        internal_ref = time.time()
        try:

            while (time.time() - self.timer_ref < 60):
                self.time_teste = time.time()
                if time.time() - internal_ref >= 0.1:
                    valor_ldr = (sensor_ldr.read()/1023.0) + 1
                    valor_ntc = (sensor_ntc.read()/1023.0) + 1
                    self.data['Temperatura'] = valor_ntc - 1 
                    self.data['Luminosidade'] = valor_ldr - 1
                    self.fila_leitura = [valor_ldr, valor_ntc]

                   # print("LDR %s" %valor_ldr)
                   # print("NTC %s" %valor_ntc)

                    pwm_lum.write(valor_ldr - 1 )
                    pwm_temp.write(valor_ntc - 1)

                    # imprime_lista(fila_leitura)
                    # fila_leitura.append(valor_ntc)
                    # print(self.fila_leitura)
                    # print("sensor 1 %s" %valor_ldr)
                    # print("sensor 2 %s" %valor_ntc)
                    internal_ref = time.time()
        except Exception as e:
            print(str(e))
            print("Algo errado na leitura. Essa porta eh certa?")

        print("Fim do sensor %s " % self.pwm_value )
        self.pwm_value = 0

    def pwm(self):
        # print(self.valor_pwm())
        pwm_motor = mraa.Pwm(3)
        pwm_motor.period_us(350)
        pwm_motor.enable(True)
        pwm_led_motor = mraa.Pwm(10)
	pwm_led_motor.period_us(350)
	pwm_led_motor.enable(True)
	internal_ref = time.time()
        pwm_motor.write(0)
        while (time.time() - self.timer_ref < 60):
            if (time.time() - internal_ref) >= 0.1:
                value = self.pwm_value / 1.0
               # print("value pwm %s "%(value/1.0))

                pwm_motor.write(value)
		pwm_led_motor.write(value)
                internal_ref = time.time()
	
        self.pwm_value = 0
	pwm_motor.write(0)
	pwm_led_motor.write(0)
        print("Fim do PWM %s " % self.pwm_value)

    def curvaSecagem(self):
        pwm_secador = 0
        self.pwm_value = 0
        print('Comecou a secar')
        seg = time.time() - self.timer_ref
        while (seg < 60):
            #self.pwm_value = pwm_secador
            seg = time.time() - self.timer_ref
            if (seg <= 10):
                pwm_secador = ((4 * seg) * (1.14 / self.fila_leitura[1]) * (1.1 / self.fila_leitura[0]))
                self.curva_original = (4 * seg)
                self.data['pwm'] = pwm_secador
                self.pwm_value = pwm_secador / 100.0
                # temp = pwm_secador
                #print(self.pwm_value)

            elif (seg <= 25):
                pwm_secador = (40 * (1.14 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])
                self.curva_original = 40
                self.data['pwm'] = pwm_secador
                # temp2 = pwm_secador
                # print(self.pwm_value)
                self.pwm_value = pwm_secador / 100.0

            elif (seg <= 35):
                pwm_secador = ((40 + ((4 * (seg - 25)) * (1.14 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])))
                self.curva_original = (40 + (4 * (seg - 25)))
                self.data['pwm'] = pwm_secador
                # temp = pwm_secador
                self.pwm_value = pwm_secador / 100.0
                # print(self.pwm_value)

            elif (seg <= 50):
                pwm_secador = (80 * (1.14 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])
                self.curva_original = 80
                self.data['pwm'] = pwm_secador
                self.pwm_value = pwm_secador / 100.0
                # temp2 = pwm_secador
                # print(self.pwm_value)

            elif (seg  <= 60):
                pwm_secador = ((80 - ((8 * (seg - 50)) * (1.4 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])))
                self.curva_original = (80 - (8 * (seg - 50)))
                self.data['pwm'] = pwm_secador
			
                if(pwm_secador < 0):
                    pwm_secador = 0
                    self.curva_original = 0
                    self.pwm_value = pwm_secador
		
                # print(self.pwm_value)
               	self.pwm_value = pwm_secador / 100.0
            else:

                self.pwm_value = 0
		pwm_secador = 0
                print('terminou de secar %s ' %  self.pwm_value)
                

    #def valor_pwm(self):
     #   self.fila_leitura[1] = (self.fila_leitura[1] / 1023) + 1
        #self.fila_leitura[0] = (self.fila_leitura[0] / 1023) + 1
        #return (selfsudo nmap -sn 10.13.110-113.255 |grep "98:4F" -B 2.pwm_value)

    def sendToFirebase(self):
        internal_ref = time.time()
        while (time.time() - self.timer_ref < 60):
            if time.time() - internal_ref >= 0.1:
    	        result = self.firebase.post('/status',{'Temperatura':self.data['Temperatura'],'Luminosidade':self.data['Luminosidade'], 
'CurvaSecagem': self.data['pwm'],'Time':(time.time() - self.timer_ref), 'original':self.curva_original})
        print("Fim envio firebase %s" % self.pwm_value)
        self.pwm_value = 0


if __name__ == '__main__':
    botao_start = mraa.Gpio(8)
    led_start = mraa.Gpio(7)
    botao_start.dir(mraa.DIR_IN)
    led_start.dir(mraa.DIR_OUT)
    estado = False

    while(True):
        if (not botao_start.read() and estado == False):
            estado = True
            led_start.write(1)

        if (estado):
            b = Secagem(0)
            t = threading.Thread(name='leitura_analogica', target=b.ler_sensores)
            w = threading.Thread(name='pwm', target=b.pwm)
            x = threading.Thread(name='secagem', target=b.curvaSecagem)
            z = threading.Thread(name='sendToFirebase', target=b.sendToFirebase)

            t.start()
            w.start()
            z.start()
            x.start()
 

            t.join()
            w.join()
            z.join()
            x.join()
            estado = False
            led_start.write(0)
            print("fim processo")
