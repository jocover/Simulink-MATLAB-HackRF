%
% Copyright 2010 Communications Engineering Lab, KIT
%
% This is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3, or (at your option)
% any later version.
%
% This software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software; see the file COPYING. If not, write to
% the Free Software Foundation, Inc., 51 Franklin Street,
% Boston, MA 02110-1301, USA.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make script for Simulink-HACKRF
% use "make -v" to get a verbose output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function make(varargin)

if (strcmp(computer('arch'),'win64'))
    arch = 'x64';
else
    arch = 'x86';
end

HACKRF_BIN_DIR = fullfile(pwd,'bin');
HACKRF_BLOCKSET_DIR = fullfile(pwd,'blockset');

if ispc
    % this should point to the directory of bladeRF.h
    HACKRF_INC_DIR = fullfile(pwd,'include');
    % this should point to the directory of bladeRF.lib
    HACKRF_LIB_DIR = fullfile(pwd,'lib',arch);
    % make sure the other required DLLS are in your PATH
    % (e.g. place them in the bin directory)

    options = { ...
        ['-I' pwd]; ...
        ['-I' HACKRF_INC_DIR]; ...
        ['-L' HACKRF_LIB_DIR]; ...
        ['-l' 'hackrf']; ...
    };
    options_pthread = { ...
        ['-l' 'pthreadVC2'] ...
    };
elseif isunix
    options = { ...
        ['-l' 'hackrf']
    };
    options_pthread = { ...
        ['-l' 'pthread'] ...
    };
else
    error('Platform not supported')
end

if (~exist(HACKRF_BIN_DIR,'dir'))
    mkdir(HACKRF_BIN_DIR);
end
if ~isempty(varargin)
    options = [options; char(varargin)];
end

% Set path hint
% compile source and find_devices
%fprintf('\nCompiling hackrf_source.cpp ... ');
%mex(options{:},options_pthread{:},'-outdir',HACKRF_BIN_DIR,'src/hackrf_source.cpp')
%fprintf('Done.\n');

%fprintf('\nCompiling hackrf_sink.cpp ... ');
%mex(options{:},options_pthread{:},'-outdir',HACKRF_BIN_DIR,'src/hackrf_sink.cpp')
%fprintf('Done.\n');

fprintf('\nCompiling hackrf_find_devices.cpp ... ');
mex(options{:},'-outdir',HACKRF_BIN_DIR,'src/hackrf_find_devices.cpp')
fprintf('Done.\n');

fprintf('\nCompiling hackrf_dev.cpp ... ');
mex(options{:},'-outdir',HACKRF_BIN_DIR,'src/hackrf_dev.cpp')
fprintf('Done.\n');

% copy help file
copyfile(fullfile(pwd,'src','hackrf_dev.m'),fullfile(HACKRF_BIN_DIR,'hackrf_dev.m'));


% Set path hint
fprintf('\nBuild successful.\n\nSet path to:\n -> %s\n -> %s\n',HACKRF_BIN_DIR,HACKRF_BLOCKSET_DIR);
