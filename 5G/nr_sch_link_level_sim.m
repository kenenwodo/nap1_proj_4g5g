%res = nr_sch_link_level_sim(frame_cfg, sim_dur_slots, UE, N_ant_eNB_RX, channel, SNR)
%
% Runs link level simulation of 5G NR PUSCH/PDSCH channel transceiver on physical
% layer level using fixed allocations of UE.
%
% Arguments:
%  frame_cfg     - OFDM framing constants structure
%  sim_dur_slots - simulation duration in slots
%  UE            - vector of structures. Each element has the following members:
%                  f_doppler - Doppler frequency in Hz
%                  mpprofile - name of the multipath channel profile
%                              see manulal of power_delay_profile function
%                  I_mcs     - MCS index
%                  N_ant_TX  - number of transmit antennas
%                  N_layer   - number of transmission layers
%                  antenna_ports - vector of antenna ports
%                  PUSCH_sched_RB_offset - index of the firt allocated PRB
%                  PUSCH_sched_RB_num    - number of scheduled PRB
%                  PUSCH_symbol_start    - index of the first allocated symbol in slot
%                  PUSCH_symbol_start    - number of allocated symbols in slot
%                  tx_filter             - vector of real-valued FIR filter coefficients
%                  higher_layer_parameters - higher layer parameter structure
%                  algorithms              - algorithms structure
%  N_ant_eNB_RX  - number of antennas in the receiver
%  channel       - structure with the following members:
%                  rayleigh_en - if set to non-zero, emulates Rayleigh fading channel
%                                if set to zero, AWGN channel is selected
%                  MIMO_corr - two element vector of MIMO channel correlation:
%                              the 1st elements is for gNB correlation, the 2nd is for UE
%                  method    - method to generate Rayleigh fading channel random variates
%                              see manual of apply_fading_td function
%                  pdp_resample_meth  - tap reduction method
%                                       see manual of power_delay_profile function
%                  pdp_reduce_N - parameter of some PDP reduction method
%                  normalize_response - if set to true, power of the channel's response is
%                                       normalized to 1  
%  SNR           - signal to noise ratio in dB
%
% Returns:
%  res           - vector of structures with simulation results (per UE)
%                  BER_c    - coded Bit Error Ratio
%                  BER_u    - uncoded Bit Error Ratio
%                  BLER     - Block Error Ratio
%                  EVM_DMRS - Error Vector Magnitude calculated based on equalized DMRS signal

% Copyright 2018 Grzegorz Cisek (grzegorzcisek@gmail.com)

function res = nr_sch_link_level_sim(frame_cfg, sim_dur_slots, UE, N_ant_eNB_RX, channel, SNR)
  for i = 1:length(UE)
    
    UE(i).PUSCH_symbols_sched_wo_DMRS = UE(i).PUSCH_symbols_sched - (UE(i).higher_layer_parameters.UL_DMRS_add_pos+1);
    UE(i).Q_m = nr_resolve_mcs(UE(i).I_mcs, UE(i).higher_layer_parameters.MCS_Table_PUSCH);
    [UE(i).tbs, UE(i).ctbs] = nr_transport_block_size(UE(i).PUSCH_symbols_sched_wo_DMRS, 0, UE(i).PUSCH_sched_RB_num, UE(i).I_mcs, UE(i).N_layer, UE(i).higher_layer_parameters.MCS_Table_PUSCH);

    UE(i).coded_tx = 0;
    UE(i).coded_err = 0;
    UE(i).uncoded_tx = 0;
    UE(i).uncoded_err = 0;
    UE(i).block_tx = 0;
    UE(i).block_err = 0;
    UE(i).EVM_meas = zeros(sim_dur_slots,1);
    
    UE(i).coded_tx1 = 0;
    UE(i).coded_err1 = 0;
    UE(i).uncoded_tx1 = 0;
    UE(i).uncoded_err1 = 0;
    UE(i).block_tx1 = 0;
    UE(i).block_err1 = 0;
    UE(i).EVM_meas1 = zeros(sim_dur_slots,1);


    [d,g] = power_delay_profile(UE(i).mpprofile, 1 / frame_cfg.F_s, channel.pdp_resample_meth, channel.pdp_reduce_N);
    UE(i).pdp = [d;g];
  end

  for n_slot = 0 : sim_dur_slots-1
    n_frame = floor(n_slot / frame_cfg.N_frame_slot);
    n_slot_frame = mod(n_slot, frame_cfg.N_frame_slot);
    
    %Connect to OPC Server
    Server=opcda('localhost','Matrikon.OPC.Simulation.1');
    hdaObj=opchda('localhost','Matrikon.OPC.Simulation.1');
    connect(Server);
    get(Server,'Status')
    
    grp_DIG = addgroup(Server);                 %Tags for RTU measurement from DigSilent
    itm1(1) = additem(grp_DIG,'Feeder 611.LOAD_611_V_RES');
    itm1(2) = additem(grp_DIG,'Feeder 611.SWT_CAP_611_RES');
    itm1(3) = additem(grp_DIG,'Feeder 611.SWT_CAP_611');
    
    k = 0 ;
    while true
        r=read(grp_DIG);
        itmIDs = {r.ItemID};
        vals=itm1.Value;
        timest= [r.TimeStamp];
        fprintf('RCC.Feeder611_LOAD_611_V_RES : %d \n', vals{1})
        fprintf('RCC.Feeder611_SWT_CAP_611_RES : %d \n', vals{2})
        fprintf('RCC.Feeder611_SWT_CAP_611 : %d \n', vals{3})

        % Transmitter_DiGSILENT 
        for i = 1:length(UE)
           %%%%%%%%%%%%%%%%%%%%%%%change the randi stuff to loop through input
           %%%%%%%%%%%%%%%%%%%%%%%bits or input the whole line %%%%%%%%%%%%%%%%%%%%%
          input1 = vals{1};
          input= fix(input1 * 10000);

          b5g = zeros(40976,1);
          convert = de2bi(input)';
          a5g = flip(convert);



          la = length(a5g);
          lb = length(b5g);
          for i5g = 1:la
              b5g(lb+1-i5g) = a5g(la+1-i5g);  
          end

          linput = logical(b5g);


          UE(i).a = double(linput);
    %        UE(i).a = randi([0 1], [UE(i).tbs 1]);

          UE(i).g = nr_sch_encode(UE(i).a, UE(i).I_mcs, UE(i).N_layer, 0, UE(i).ctbs, UE(i).higher_layer_parameters.MCS_Table_PUSCH);
          x_tx = nr_pusch_transmit(UE(i).g, UE(i).Q_m, UE(i).N_layer, frame_cfg, n_slot_frame, UE(i).PUSCH_symbol_start, UE(i).PUSCH_sched_RB_offset, UE(i).PUSCH_sched_RB_num, UE(i).antenna_ports, UE(i).higher_layer_parameters, 0, 0);
          UE(i).y_tx = nr_ofdma_modulator(x_tx, frame_cfg, n_slot_frame);
          for n_tx = 1 : size(UE(i).y_tx,2)
            UE(i).y_tx(:,n_tx) = conv(UE(i).y_tx(:,n_tx), UE(i).tx_filter', 'same');
          end

          transmitted = input;
          fprintf('Transimtted integer value from DiGSILENT: %d.\n',transmitted);

        end

        % Wireless Channel_DiGSILENT_to_Control Center
        y_tx = zeros(size(UE(1).y_tx));
        for i = 1:length(UE)
          if channel.rayleigh_en
            y_tx_ue = apply_fading_td(UE(i).y_tx, UE(i).f_doppler, frame_cfg.F_s, UE(i).pdp, channel.method, [UE(i).N_ant_TX, N_ant_eNB_RX, channel.MIMO_corr(1), channel.MIMO_corr(2)]);
          else
            y_tx_ue = UE(i).y_tx;
          end

          if channel.normalize_response
            for n_tx = 1 : size(y_tx_ue,2)
              y_tx_ue(:,n_tx) = y_tx_ue(:,n_tx) / rms(y_tx_ue(:,n_tx)) * rms(UE(i).y_tx(:,n_tx));
            end
          end

          y_tx = y_tx + cfo_add(y_tx_ue, channel.F_cfo, frame_cfg.F_s);
          y_tx = y_tx + y_tx_ue;
        end

        noise = 10.0 ^ (-SNR / 20.0) / sqrt(2) * (randn(size(y_tx)) + 1i * randn(size(y_tx)));
        y_rx = y_tx + noise;

        % Receiver_Control Center
        for i = 1:length(UE)
          x_rx = nr_ofdma_demodulator(y_rx, frame_cfg, n_slot_frame);
          [llrs, EVM_DMRS] = nr_pusch_receive(x_rx, UE(i).Q_m, UE(i).N_layer, frame_cfg, n_slot_frame, UE(i).PUSCH_symbol_start, UE(i).PUSCH_symbols_sched, UE(i).PUSCH_sched_RB_offset, UE(i).PUSCH_sched_RB_num, UE(i).antenna_ports, UE(i).higher_layer_parameters, UE(i).algorithms, 0);      
          [a_rx, ~, cb_crc_ok] = nr_sch_decode(llrs, UE(i).I_mcs, UE(i).N_layer, 0, UE(i).tbs, UE(i).higher_layer_parameters.MCS_Table_PUSCH);
           %%%%%%%%%%%%%%%%%%%%%%%  a_rx is received binary vector **********

          received = bi2de(a_rx','left-msb');
          %receivedbits = (flip(de2bi(received)))';
          fprintf('Received integer value at CC: %d.\n',received);
          
          %Control action taken at the Control Center
          x= received/24017;
          volt_diff= 1-x;
          if volt_diff>= 0.05
            y=1;   
          else 
            y=0;
          end
      
          % update statistics
          UE(i).coded_tx = UE(i).coded_tx + numel(a_rx);
          UE(i).coded_err = UE(i).coded_err + sum(a_rx ~= UE(i).a);

          UE(i).uncoded_tx = UE(i).uncoded_tx + numel(llrs);
          UE(i).uncoded_err = UE(i).uncoded_err + sum(llr2hardbit(llrs) ~= UE(i).g(:));

          UE(i).block_tx = UE(i).block_tx + numel(cb_crc_ok);
          UE(i).block_err = UE(i).block_err + numel(cb_crc_ok) - sum(cb_crc_ok);

          UE(i).EVM_meas(n_slot+1) = rms(EVM_DMRS(:));
        end
    
     % Transmitter_Control Center 
        for i = 1:length(UE)
           %%%%%%%%%%%%%%%%%%%%%%%change the randi stuff to loop through input
           %%%%%%%%%%%%%%%%%%%%%%%bits or input the whole line %%%%%%%%%%%%%%%%%%%%%
          input2 = y;
          

          b5g = zeros(40976,1);
          convert = de2bi(input2)';
          a5g = flip(convert);



          la = length(a5g);
          lb = length(b5g);
          for i5g = 1:la
              b5g(lb+1-i5g) = a5g(la+1-i5g);  
          end

          linput = logical(b5g);


          UE(i).a = double(linput);
    %        UE(i).a = randi([0 1], [UE(i).tbs 1]);

          UE(i).g = nr_sch_encode(UE(i).a, UE(i).I_mcs, UE(i).N_layer, 0, UE(i).ctbs, UE(i).higher_layer_parameters.MCS_Table_PUSCH);
          x_tx = nr_pusch_transmit(UE(i).g, UE(i).Q_m, UE(i).N_layer, frame_cfg, n_slot_frame, UE(i).PUSCH_symbol_start, UE(i).PUSCH_sched_RB_offset, UE(i).PUSCH_sched_RB_num, UE(i).antenna_ports, UE(i).higher_layer_parameters, 0, 0);
          UE(i).y_tx = nr_ofdma_modulator(x_tx, frame_cfg, n_slot_frame);
          for n_tx = 1 : size(UE(i).y_tx,2)
            UE(i).y_tx(:,n_tx) = conv(UE(i).y_tx(:,n_tx), UE(i).tx_filter', 'same');
          end

          transmitted = input2;
          fprintf('Transimtted integer value from Control Center: %d.\n',transmitted);

        end

        % Wireless Channel_Control Center_to_DiGSILENT
        y_tx = zeros(size(UE(1).y_tx));
        for i = 1:length(UE)
          if channel.rayleigh_en
            y_tx_ue = apply_fading_td(UE(i).y_tx, UE(i).f_doppler, frame_cfg.F_s, UE(i).pdp, channel.method, [UE(i).N_ant_TX, N_ant_eNB_RX, channel.MIMO_corr(1), channel.MIMO_corr(2)]);
          else
            y_tx_ue = UE(i).y_tx;
          end

          if channel.normalize_response
            for n_tx = 1 : size(y_tx_ue,2)
              y_tx_ue(:,n_tx) = y_tx_ue(:,n_tx) / rms(y_tx_ue(:,n_tx)) * rms(UE(i).y_tx(:,n_tx));
            end
          end

          y_tx = y_tx + cfo_add(y_tx_ue, channel.F_cfo, frame_cfg.F_s);
          y_tx = y_tx + y_tx_ue;
        end

        noise = 10.0 ^ (-SNR / 20.0) / sqrt(2) * (randn(size(y_tx)) + 1i * randn(size(y_tx)));
        y_rx = y_tx + noise;
        
        
        % Receiver_DiGSILENT
        for i = 1:length(UE)
          x_rx = nr_ofdma_demodulator(y_rx, frame_cfg, n_slot_frame);
          [llrs, EVM_DMRS] = nr_pusch_receive(x_rx, UE(i).Q_m, UE(i).N_layer, frame_cfg, n_slot_frame, UE(i).PUSCH_symbol_start, UE(i).PUSCH_symbols_sched, UE(i).PUSCH_sched_RB_offset, UE(i).PUSCH_sched_RB_num, UE(i).antenna_ports, UE(i).higher_layer_parameters, UE(i).algorithms, 0);      
          [a_rx, ~, cb_crc_ok] = nr_sch_decode(llrs, UE(i).I_mcs, UE(i).N_layer, 0, UE(i).tbs, UE(i).higher_layer_parameters.MCS_Table_PUSCH);
           %%%%%%%%%%%%%%%%%%%%%%%  a_rx is received binary vector **********

          received = bi2de(a_rx','left-msb');
          %receivedbits = (flip(de2bi(received)))';
          fprintf('Received integer value at DiGSILENT: %d.\n',received);
          write(itm1(3),received);
      
          % update statistics
          UE(i).coded_tx1 = UE(i).coded_tx1 + numel(a_rx);
          UE(i).coded_err1 = UE(i).coded_err1 + sum(a_rx ~= UE(i).a);

          UE(i).uncoded_tx1 = UE(i).uncoded_tx1 + numel(llrs);
          UE(i).uncoded_err1 = UE(i).uncoded_err1 + sum(llr2hardbit(llrs) ~= UE(i).g(:));

          UE(i).block_tx1 = UE(i).block_tx1 + numel(cb_crc_ok);
          UE(i).block_err1 = UE(i).block_err1 + numel(cb_crc_ok) - sum(cb_crc_ok);

          UE(i).EVM_meas1(n_slot+1) = rms(EVM_DMRS(:));
        end
      
          res = struct();

          res.BER_c     = zeros(length(UE), 1);
          res.BER_u     = zeros(length(UE), 1);
          res.BLER      = zeros(length(UE), 1);
          res.EVM_DMRS  = zeros(length(UE), 1);

        for i = 1:length(UE)
        res.BER_c   (i) = UE(i).coded_err / UE(i).coded_tx;
        res.BER_u   (i) = UE(i).uncoded_err / UE(i).uncoded_tx;
        res.BLER    (i) = UE(i).block_err / UE(i).block_tx;
        res.EVM_DMRS(i) = rms(UE(i).EVM_meas);
        end
        
        
        res1 = struct();

          res1.BER_c     = zeros(length(UE), 1);
          res1.BER_u     = zeros(length(UE), 1);
          res1.BLER      = zeros(length(UE), 1);
          res1.EVM_DMRS  = zeros(length(UE), 1);

        for i = 1:length(UE)
        res1.BER_c   (i) = UE(i).coded_err1 / UE(i).coded_tx1;
        res1.BER_u   (i) = UE(i).uncoded_err1 / UE(i).uncoded_tx1;
        res1.BLER    (i) = UE(i).block_err1 / UE(i).block_tx1;
        res1.EVM_DMRS(i) = rms(UE(i).EVM_meas1);
        end
        
        fprintf('Error values for data sent from DiGSILENT to CC: \n');
        disp(res)
        fprintf('Error values for data sent from CC to DiGSILENT: \n');
        disp(res1)
    k = k + 1 ;
    if k > 15
        break
    end
    disp(k)
end
  
 
end