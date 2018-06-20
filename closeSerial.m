function closeSerial()

    if ~isempty(instrfind);
        fclose(instrfind);
        delete(instrfind);
    end
    disp('Serial Port Closed')

end