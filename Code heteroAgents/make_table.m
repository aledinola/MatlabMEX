function [] = make_table(ResFolder,Outputs,Params)

disp(' ')
disp('REPLICATE TABLE 1 OF BUERA AND SHIN (2013)')

us_data = [0.67,0.30,0.10,0.045];
model1  = us_data;
model2  = [Outputs.top10_empl,Outputs.top5_earnings,Outputs.exit_E_to_W,Outputs.r];

fprintf(' %-30s %-10s %-10s %-10s\n', '', 'US Data', 'Model BS', 'Model Replication');
fprintf(' %s\n', repmat('-', 1, 70));
fprintf(' %-30s %-10.4f %-10.4f %-10.4f\n', ...
    'Top 10% Employment', us_data(1), model1(1), model2(1));
fprintf(' %-30s %-10.4f %-10.4f %-10.4f\n', ...
    'Top 5% Earnings', us_data(2), model1(2), model2(2));
fprintf(' %s\n', repmat('-', 1, 70));
fprintf(' %-30s %-10.4f %-10.4f %-10.4f\n', ...
    'Establishment Exit Rate', us_data(3), model1(3), model2(3));
fprintf(' %-30s %-10.4f %-10.4f %-10.4f\n', ...
    'Real Interest Rate', us_data(4), model1(4), model2(4));
disp(' ')

fid=fopen(fullfile(ResFolder,'targets_model_manual.txt'),'wt');  % overwrite

fprintf(fid,"MODEL PARAMETERS \n");
fprintf(fid,"beta         = %f \n",Params.beta);
fprintf(fid,"eta          = %f \n",Params.eta);
fprintf(fid,"psi          = %f \n",Params.psi);
fprintf(fid,"span_control = %f \n",1-Params.upsilon);
fprintf(fid,"alpha        = %f \n",Params.alpha);
fprintf(fid,"delta        = %f \n",Params.delta);
fprintf(fid,"lambda       = %f \n",Params.lambda);

fprintf(fid,"TARGETED MOMENTS \n");
fprintf(fid,"Top 10 Employment = %f \n",Outputs.top10_empl);
fprintf(fid,"Top 5 Earnings    = %f \n",Outputs.top5_earnings);
fprintf(fid,"Entre exit rate   = %f \n",Outputs.exit_E_to_W);
fprintf(fid,"Interest rate     = %f \n",Outputs.r);
fprintf(fid,"GE cond 1   = %f \n",Outputs.GE_eqns_vec(1));
fprintf(fid,"GE cond 2   = %f \n",Outputs.GE_eqns_vec(2));

fprintf(fid,"OTHER MOMENTS \n");
fprintf(fid,"Share of entre    = %f \n",Outputs.share_entre);
fprintf(fid,"K/Y ratio         = %f \n",Outputs.K_Y);
fprintf(fid,"ExtFin/Y ratio    = %f \n",Outputs.extfin_Y);
fprintf(fid,"K    = %f \n",Outputs.K);
fprintf(fid,"L    = %f \n",Outputs.L);
fprintf(fid,"Y    = %f \n",Outputs.Y);
fprintf(fid,"w    = %f \n",Outputs.w);

fclose(fid);

fprintf('File %s written to %s \n','targets_model_manual.txt',ResFolder)

end %end function
