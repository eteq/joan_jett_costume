import time
import board
import digitalio
import audiobusio
import audiocore

import array,math


led = digitalio.DigitalInOut(board.LED)
led.switch_to_output()
led.value = False

g = digitalio.DigitalInOut(board.A4)
g.switch_to_output()
g.value = True
sw = digitalio.DigitalInOut(board.A5)
sw.switch_to_input(digitalio.Pull.DOWN)

i2s = audiobusio.I2SOut(bit_clock=board.D1, word_select=board.D7, data=board.D9)


sine_wave = False

if sine_wave:
    length = 8000 // 440
    sine_wave = array.array("H", [0] * length)
    for i in range(length):
        sine_wave[i] = int(math.sin(math.pi * 2 * i / length) * (2 ** 15) + 2 ** 15)
    audio = audiocore.RawSample(sine_wave, sample_rate=8000)
else:
    f = open('badrep.wav', 'rb')
    audio = audiocore.WaveFile(f)

while True:
    if sw.value:
        led.value = True
        if not i2s.playing:
            i2s.play(audio, loop=True)

    else:
        led.value = False
        if i2s.playing:
            i2s.stop()

