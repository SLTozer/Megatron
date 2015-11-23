function a=md5(varargin)

% USAGE
%      a=md5(varargin)
%       
%   where the arguements are filenames.
%   If only one filename is specified (if number of input arguements is 1), 
%       the MD5 sum of the file is returned
%   If two filenames are specified (if number of input arguements is 2),
%       the MD5 sum of the two files are compared and
%       1 is returned if they are the same
%       0 is returned if they are different
%
%   This program needs MD5 Utility 
%       This can be got from http://www.fourmilab.ch/md5/
%       and lots of other sites, but parameters might be different 
%       for other versions. This program uses the md5.exe downloaded
%       from the above site. 
%
%   NOTE: Change md5path (line 24) in this program to the path where md5.exe is present

%Suresh E Joel, May 12,2003

md5path= strcat('"',pwd,'\');

switch(nargin)
case 0,
    error('Too few arguements for MD5');
case 1
    s=sprintf('%c%s%s%s%s\n','!',md5path,'md5.exe" -n -otempmd5.txt "',strtrim(varargin{1}), '"');
    eval(s);
    a=textread('tempmd5.txt','%s');
case 2
    s=sprintf('%c%s%s%s%s\n','!',md5path,'md5.exe" -n -otempmd5.txt "',strtrim(varargin{1}), '"');
    eval(s);
    a=textread('tempmd5.txt','%s');
    s=sprintf('%c%s%s%s%s\n','!',md5path,'md5.exe" -n -otempmd5.txt "',strtrim(varargin{2}), '"');
    eval(s);
    b=textread('tempmd5.txt','%s');
    a=strcmp(a,b);
otherwise
    error('Too many arguements for MD5');
end;