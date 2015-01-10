% Generate a 1 kHz sine waveform with a 10 seconds duration
Fs = 44.1e3; % one of the standard sampling freq. for audio signals
Tm = 60; % max duration in seconds
Nm = Fs*Tm; % the max number of sampling points
t = (0:Nm)/Fs; % t-axis
A0 = 1; % Amplitude of the signal
f0 = 1e3; % frequency of the signal in Hz
x_t = A0*sin(2*pi*f0*t);
nn = 1:200; plot(t(nn),x_t(nn))
sound(x_t, Fs); %sends x_t with FS out to the speaker. Note -1.0<=x_t<=1.0