%clear all
%close all
function WAVEFILE=dial()
	disp('Dial the number ');

	Phone_nbr_in = input('Phone_nbr: ','s'); %The input of Phone_nbr
	Phone_nbr_in; % Data type must be of type string, otherwise a number starting with a zero be false

	[wav0,FS,NBITS] = wavread('dtmf-0.wav');
	[wav1,FS,NBITS] = wavread('dtmf-1.wav');
	[wav2,FS,NBITS] = wavread('dtmf-2.wav');
	[wav3,FS,NBITS] = wavread('dtmf-3.wav');
	[wav4,FS,NBITS] = wavread('dtmf-4.wav');
	[wav5,FS,NBITS] = wavread('dtmf-5.wav');
	[wav6,FS,NBITS] = wavread('dtmf-6.wav');
	[wav7,FS,NBITS] = wavread('dtmf-7.wav');
	[wav8,FS,NBITS] = wavread('dtmf-8.wav');
	[wav9,FS,NBITS] = wavread('dtmf-9.wav');
	[wavstar,FS,NBITS] = wavread('dtmf-star.wav');
	[wavhash,FS,NBITS] = wavread('dtmf-hash.wav');

	i=1;
	tones=[];

while size(Phone_nbr_in,2)>=i

	number=Phone_nbr_in(i);
    		switch number 
       			case {num2str(0)}
            			tones=[tones;wav0];
       
       			case {num2str(1)}
            			tones=[tones;wav1];
       
       			case {num2str(2)}
            			tones=[tones;wav2];
      
       			case {num2str(3)}
            			tones=[tones;wav3];
       
       			case {num2str(4)}
            			tones=[tones;wav4];
       
       			case {num2str(5)}
		        	tones=[tones;wav5];
       
       			case {num2str(6)}
            			tones=[tones;wav6];
       
       			case {num2str(7)}
            			tones=[tones;wav7];
       
       			case {num2str(8)}
            			tones=[tones;wav8];
       
       			case {num2str(9)}
            			tones=[tones;wav9];
            
        		case{35}
        			tones=[tones;wavhash];
       
       			case{42}
       				tones=[tones;wavstar];
       
       			otherwise
       				warning('An incorrect character has been given') 
       				Phone_nbr_in(i)
    		end
	i=i+1;
end 

FS 
NBITS
WAVEFILE='dialtones.wav';
wavwrite(tones,FS,NBITS,WAVEFILE);
sound(tones,FS);

