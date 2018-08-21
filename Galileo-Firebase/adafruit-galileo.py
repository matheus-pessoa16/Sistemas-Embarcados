# -*- coding: utf-8 -*-

import threading
import time
import mraa
from Adafruit_IO import Client


class Secagem():
    def __init__(self):
        self.fila_leitura = [1, 1]
        self.timer_ref = time.time()
        self.pwm_value = 1 
        self.data = {}
        self.aio = Client('fdaf965e65624e049d87ee2cc6fd6640')

    def ler_sensores(self):
        sensor_ldr = mraa.Aio(0)
        sensor_ntc = mraa.Aio(1)
        # print("Portas analogicas setadas")
        internal_ref = time.time()
        try:

            while (time.time() - self.timer_ref < 60):
                if time.time() - internal_ref >= 0.1:
                    valor_ldr = sensor_ldr.read()
                    valor_ntc = sensor_ntc.read()
                    self.data['Temperatura'] = valor_ntc
                    self.data['Luminosidade'] = valor_ldr
                    #self.fila_leitura = [valor_ldr, valor_ntc]
                    # imprime_lista(fila_leitura)
                    # fila_leitura.append(valor_ntc)
                    # print(self.fila_leitura)
                    # print("sensor 1 %s" %valor_ldr)
                    # print("sensor 2 %s" %valor_ntc)
                    internal_ref = time.time()
        except:
            print("Algo errado na leitura. Essa porta eh certa?")

    def pwm(self):
        # print(self.valor_pwm())
        pwm_motor = mraa.Pwm(3)
        pwm_motor.period_us(10)
        pwm_motor.enable(True)
        internal_ref = time.time()
        while (time.time() - self.timer_ref < 60):
            if (time.time() - internal_ref) >= 0.1:
                value = self.valor_pwm
                # print(value)
                self.data['CurvaSecagem'] = self.pwm_value
                pwm_motor.write(value)
                internal_ref = time.time()

    def curvaSecagem(self):
        intial_time = time.time()  # começa a contagem da curva de secagem
        pwm_secador = 0
        seg = time.time() - intial_time
        while (seg < 60):
            self.pwm_value = pwm_secador
            seg = time.time() - intial_time
            if (seg <= 10):
                pwm_secador = ((4 * seg) * (1.14 / self.fila_leitura[1]) * (1.1 / self.fila_leitura[0]))
                #pwm_value = pwm_secador / 100.0
                # temp = pwm_secador
                #print(pwm_secador)


            elif (seg <= 25):
                pwm_secador = (40 * (1.14 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])
                # temp2 = pwm_secador
                #print(pwm_secador)
                #pwm_value = pwm_secador / 100.0

            elif (seg <= 35):
                pwm_secador = ((40 + ((4 * (seg - 25)) * (1.14 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])))
                # temp = pwm_secador
                #pwm_value = pwm_secador / 100.0
                #print(pwm_secador)

            elif (seg <= 50):
                pwm_secador = (80 * (1.14 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])
                #pwm_value = pwm_secador / 100.0
                # temp2 = pwm_secador
                #print(pwm_secador)

            elif (time.time() - intial_time <= 60):
                pwm_secador = ((100 - ((8 * (seg - 50)) * (1.4 / self.fila_leitura[1])) * (1.1 / self.fila_leitura[0])))
                #print(pwm_secador)
                #pwm_value = pwm_secador / 100.0
            else:
                seg = time.time() - intial_time


    def valor_pwm(self):
        self.fila_leitura[1] = (self.fila_leitura[1] / 1023) + 1
        self.fila_leitura[0] = (self.fila_leitura[0] / 1023) + 1
        return (self.pwm_value)


    def sendToAdafruitIO(self):
        internal_ref = time.time()
        while True:
            if time.time() - internal_ref >= 0.2:
                self.aio.send('Temperatura', self.data['Temperatura'])
                #self.aio.send(data['Processo'])
                self.aio.send('Luminosidade', self.data['Luminosidade'])
                self.aio.send('CurvaSecagem', self.pwm_value)
                internal_ref = time.time()



if __name__ == '__main__':
    b = Secagem()
    t = threading.Thread(name='leitura_analogica', target=b.ler_sensores)
    w = threading.Thread(name='pwm', target=b.pwm)
    x = threading.Thread(name='secagem', target=b.curvaSecagem)
    z = threading.Thread(name='sendToAdafruitIO', target=b.sendToAdafruitIO)

    t.start()
    w.start()
    x.start()
    z.start()

    t.join()
    w.join()
    x.join()
    z.join()
