"""
Interactive piano steps.

- Timidity package must be installed on the playback device, used to provide
software midi synth. (Default soundfont installed contains the piano sample)
  NOTE: If you want to play more complex midi tracks through timidity you must 
  provide an alternative soundfont. Try fluid-soundfont-gm or copy over GM.DLS
  from a Windows PC, then configure timidity to use the new soundfont. Test with
  a complex track (onestop.mid works well for using many instruments)
  This is just FYI, we only need a piano sound which comes by default

- Timidity service must be running.
  https://wiki.archlinux.org/index.php/timidity#Daemon 
  (Don't be tempted to install the timidity-daemon package, it assumes sysv init
  when debian has long moved to systemd)

- The python-rtmidi pip package is installed.
  You will need the libasound2-dev and libjack-dev system packages for this as
  well. Some systems have problems installing libjack-dev so try installing
  libjack-jackd2-dev in its place.

- The mido pip package is installed.

Limitations:

"""

import Adafruit_ADS1x15
import time
import mido
import json
import os
from concurrent import futures
import threading


class BaseProxSensor:
    def __init__(self, adc: Adafruit_ADS1x15.ADS1115, channel: int):
        self.adc = adc
        self.channel = channel
    
    @property
    def i2c_address(self) -> int:
        return self.adc._device._address 

    def poll_device(self):
        raise NotImplementedError
    
    def is_key_activated(self) -> bool:
        raise NotImplementedError


class ProxSensor(BaseProxSensor):
    # SHARP sensor-specific details
    
    # gradient of inverse-distance vs voltage graph
    pred_gradient = 141.59

    # y-intercept of inverse-distance vs voltage graph
    pred_coef = 1.1111

    # minimum detectable distance
    distance_min = 100

    # maximum detectable distance
    distance_max = 500

    def __init__(self, adc: Adafruit_ADS1x15.ADS1115, channel: int,
            threshold: int, gain: float, fuzz: bool):
        """
        Create a new proximity sensor object.

        adc: Instance of Adafruit_ADS1x15.ADS1115
        channel: Channel number of ADC to listen on
        threshold: minimum raw ADC value to trigger the sensor on (0-32767)
        gain: ADS gain setting 
        fuzz: Delay mode
        """
        self.adc = adc
        self.channel = channel
        self.threshold = threshold
        self.cached_poll = 0
        self.gain = gain
        self.fuzz = fuzz
    
    def __str__(self) -> str:
        return '<ProxSensor 0x{:x}/{} last reading {}>'.format(
            self.i2c_address, self.channel, self.cached_poll
        )
    
    def poll_device(self) -> int:
        """
        Poll the proximity sensor on this address and channel
        Reading value 0-32767 corresponds to voltage range 0-4V
        The actual voltage range for the sensor is 1.35V-3V
        Returns a 16 bit signed int

        Infinity or point blank: 12-13k
        """
        self.cached_poll = self.adc.read_adc(self.channel, gain=self.gain)
        return self.cached_poll

    def is_key_activated(self) -> bool:
        """
        Check if the sensor is in the active state. 
        """
        # return self.is_obstruction_in_target_range(
        #     self.raw_to_distance(
        #         self.poll_device()
        #     )
        # )
        if self.fuzz:
            cached_raw = self.cached_poll
            raw = self.poll_device()
            # print(str(self))

            # if raw > self.threshold:
            #     if cached_raw > self.threshold:

            # else:
            #     pass

            return (raw > self.threshold) and (cached_raw > self.threshold)
        else:
            raw = self.poll_device()
            # print(str(self))
            return raw > self.threshold


class ProxSensorExpFilter(BaseProxSensor):
    def __init__(self, adc: Adafruit_ADS1x15.ADS1115, channel: int,
            threshold: int, gain: float, alpha: float, initial: float=1.0):
        """
        Create a new proximity sensor object with exponential filtering.

        adc: Instance of Adafruit_ADS1x15.ADS1115
        channel: Channel number of ADC to listen on
        threshold: minimum raw ADC value to trigger the sensor on (0-32767)
        gain: ADS gain setting 
        alpha: Exponential filter constant (in interval (0,1],
            higher -> faster response but also noisier)
        initial: Initial value for exponential filter
        """
        self.adc = adc
        self.channel = channel
        self.threshold = threshold
        self.last_value = initial
        self.gain = gain
        assert 0 < alpha <= 1
        self.alpha = alpha
    
    def __str__(self) -> str:
        return '<ProxSensorExpFilter 0x{:x}/{} last_value {:.0f}>'.format(
            self.i2c_address, self.channel, self.last_value
        )

    def poll_device(self) -> float:
        """
        Poll the proximity sensor on this address and channel
        Reading value 0-32767 corresponds to voltage range 0-4V
        The actual voltage range for the sensor is 1.35V-3V
        Returns a 16 bit signed int

        Infinity or point blank: 12-13k
        """
        raw = self.adc.read_adc(self.channel, gain=self.gain, data_rate=860)
        self.last_value = (self.alpha * raw) \
                          + ((1 - self.alpha) * self.last_value)
        return self.last_value

    def is_key_activated(self) -> bool:
        """
        Check if the sensor is in the active state. 
        """
        return self.poll_device() > self.threshold


class BaseMidi:
    def create(self):
        raise NotImplementedError
    
    def start(self):
        raise NotImplementedError
    
    def stop(self):
        raise NotImplementedError


class MidiCommandOneshot(BaseMidi):
    def __init__(self, port: mido.ports.BaseOutput,
            channel: int, program: int, note: str, velocity: int):
        """
        Create a new MidiCommandOneshot for a note that plays and decays
        immediately when triggered.
        """
        self.port = port
        self.channel = channel
        self.program = program
        self.note = note
        self.velocity = velocity

        self.created = False

    
    def create(self):
        self.port.send(mido.Message('program_change', channel=self.channel,
            program=self.program))
        self.note_on = mido.Message('note_on', channel=self.channel,
            note=self.note)
        self.note_off = mido.Message('note_off', channel=self.channel,
            note=self.note, velocity=0)
        self.created = True
    
    def start(self):
        """
        Start the midi note sequence.
        For this implementation, play the note and immediately decay.
        """
        if not self.created:
            raise RuntimeError('Call .create() method first!')
        
        self.port.send(self.note_on)
        self.port.send(self.note_off)
    
    def stop(self):
        # Do nothing
        pass


class MidiCommandOneshotDelay(BaseMidi):
    def __init__(self, port: mido.ports.BaseOutput,
            channel: int, program: int, note: str, velocity: int,
            executor: futures.Executor, delay: int):
        """
        Create a new MidiCommandOneshot for a note that plays and decays
        after a second when triggered.

        This class has executor and delay parameters, for a ThreadPoolExecutor 
        or ProcessPoolExecutor and a time delay to send the note_off command.

        Only 1 thread should call methods on this class at any time!
        """
        self.port = port
        self.channel = channel
        self.program = program
        self.note = note
        self.velocity = velocity
        self.delay = delay
        self.executor = executor

        self.created = False
        self.playing = False

    def __str__(self) -> str:
        return '<MidiCommandOneshotDelay: note {}, delay {}>'.format(
            self.note, self.delay
        )
    
    def create(self):
        self.port.send(mido.Message('program_change', channel=self.channel,
            program=self.program))
        self.note_on = mido.Message('note_on', channel=self.channel,
            note=self.note)
        self.note_off = mido.Message('note_off', channel=self.channel,
            note=self.note, velocity=0)
        self.created = True
    
    def start(self):
        """
        Start the midi note sequence.
        For this implementation, play the note and decay after a second.
        """
        # no, can't just time.sleep(1) after note_on
        if not self.created:
            raise RuntimeError('Call .create() method first!')
        
        if self.playing:
            print('Tried to double-trigger me: channel {}'.format(self.channel))
            return

        def runner(self):    
            self.playing = True
            self.port.send(self.note_on)
            # This sleeps in another thread and sends the note_off afterwards.
            time.sleep(self.delay)
            self.port.send(self.note_off)
            self.playing = False
        
        self.executor.submit(runner, self)
    
    def stop(self):
        # Do nothing
        pass


class PianoKeyManager():
    # STATE_OFF = 0
    # STATE_TURNING_ON = 1
    # STATE_ON = 2
    # STATE_TURNING_OFF = 3

    def __init__(self, sensor: BaseProxSensor, note: BaseMidi):
        """
        Create a new piano key manager object that brings together the sensor
        and the midi features. This object remembers the state of the key so 
        that the midi note doesn't retrigger every time the sensor is polled.

        Fuzzing of the input would be implemented here
        """
        # self.state = self.STATE_OFF
        self.state = 'off'
        self.sensor = sensor
        self.note = note
        # initialize the midi channel
        self.note.create()
    
    def __str__(self) -> str:
        return '<PianoKeyManager: sensor {}, midi {}>'.format(
            self.sensor, self.note)
    
    # printing a list calls repr instead of str on its elements, and we want 
    # the master list of PianoKeyManagers to look nice when we print it
    __repr__ = __str__
        
    def tick_key(self):
        """
        If the sensor transitions from inactive to active, play the midi note.
        If the sensor transitions from active to inactive, stop the midi note
        (no effect in the default oneshot implementation).
        """
        sensor_on = self.sensor.is_key_activated()

        if self.state == 'off':
            if sensor_on:
                print('{}: transitioning to ON'.format(self.sensor))
                self.state = 'on'
                self.note.start()
        elif self.state == 'on':
            if not sensor_on:
                print('{}: transitioning to OFF'.format(self.sensor))
                self.state = 'off'
                self.note.stop()
    

#######################
#  F U N C T I O N S  #
#######################


def test_polyphony():
    TIMIDITY_PORT_0 = 'TiMidity port 0'
    try:
        port = mido.open_output(TIMIDITY_PORT_0)
    except IOError:
        # Probably timidity service isn't running.
        print('Starting timidity daemon (you\'ll see timidity console output '
            'here)')
        os.system('timidity -iA &')
        port = mido.open_output(TIMIDITY_PORT_0)

    # thread pool to send note_off commands after a delay, without locking up
    # the main thread
    thread_pool = futures.ThreadPoolExecutor(max_workers=8)

    midi_commands = [
        MidiCommandOneshotDelay(
            port=port,
            channel=0,
            program=0,
            note=60,
            velocity=64,
            executor=thread_pool,
            delay=1.5
        ),
        MidiCommandOneshotDelay(
            port=port,
            channel=1,
            program=0,
            note=64,
            velocity=64,
            executor=thread_pool,
            delay=1.25
        ),
        MidiCommandOneshotDelay(
            port=port,
            channel=2,
            program=0,
            note=67,
            velocity=64,
            executor=thread_pool,
            delay=1
        )
    ]
    
    # play a nice C major
    for i in midi_commands:
        i.create()
    
    print('Playing C major')
    for i in midi_commands:
        i.start()


def profile_i2c():
    with open('piano_conf.json') as f:
        conf = json.load(f)
    
    # try to use all configured ADS
    ads = {
        int(k, base=0): Adafruit_ADS1x15.ADS1115(**v)
        for k, v in conf['ads'].items()
        if k in conf['ads_enabled']
    }

    def worker(info):
        while info[0]:
            try:
                for a in ads.values():
                    for i in range(4):
                        a.read_adc(i, data_rate=860)
            except IOError:
                pass
            info[1] += 1
    
    info = [True, 0]
    thread = threading.Thread(target=worker, args=(info,))
    thread.start()
    start_time = time.perf_counter()

    time.sleep(5)
    info[0] = False
    thread.join()
    end_time = time.perf_counter()

    delta = end_time - start_time
    rate = info[1] / delta

    print('cycles: {} time taken: {:.5f} rate: {:.5f}'.format(
        info[1], delta, rate
    ))
    

def main():
    # Load the config file
    with open('piano_conf.json') as f:
        conf = json.load(f)
    
    TIMIDITY_PORT_0 = 'TiMidity port 0'
    try:
        port = mido.open_output(TIMIDITY_PORT_0)
    except IOError:
        # Probably timidity service isn't running.
        print('Starting timidity daemon (you\'ll see timidity console output '
            'here)')
        os.system('timidity -iA &')
        time.sleep(1)
        port = mido.open_output(TIMIDITY_PORT_0)

    ads = {
        int(k, base=0): Adafruit_ADS1x15.ADS1115(**v)
        for k, v in conf['ads'].items()
        if k in conf['ads_enabled']
    }

    print('ADS in use: {}'.format(ads))

    # thread pool to send note_off commands after a delay, without locking up
    # the main thread
    thread_pool = futures.ThreadPoolExecutor(max_workers=8)

    # create our sensor-midi objects
    piano_keys = []
    for i, c in enumerate(conf['keys']):
        if i not in conf['keys_enabled']:
            # skip adding this key
            continue

        sensor = ProxSensorExpFilter(
            ads[c['sensor']['i2c_address']],
            c['sensor']['channel'],
            c['sensor']['threshold'],
            conf['gain'],
            c['sensor']['alpha']
        )

        midi = MidiCommandOneshotDelay(
            port,
            executor=thread_pool,
            delay=conf['note_length'],
            **c['midi']
        )

        piano_keys.append(PianoKeyManager(sensor, midi))
    
    print('Keys in use: {}'.format(piano_keys))

    ioerror_times = 0

    while True:
        # main loop and error handling
        try:
            for key in piano_keys:
                key.tick_key()
            ioerror_times = 0
            time.sleep(conf['tick_delay'])
        except IOError as e:
            ioerror_times += 1
            if ioerror_times % 10 == 0:
                print("IOError (i2c bus error?) {} in a row on key {} ({})".format(
                    ioerror_times, key.sensor, repr(e)))
            time.sleep(conf['ioerror_delay'])
            # swallow the error and keep going
        except KeyboardInterrupt:
            # must close the midi port!
            port.close()
            thread_pool.shutdown(wait=False)
            print('Thanks for flying with Piano Steps')
            raise

if __name__ == '__main__':
    main()
