function path = PSDMDir
    % PSDMDIR Returns the path to the current PSDM Directory.

    path = fullfile( fileparts(mfilename('fullpath')), '..');