function share = percentile_share(values, weights, pct_cut)
% Safe calculation of top-share (e.g. top 10% or top 5%) values by weight
% Inputs:
%   values   - N×1 vector (labor or earnings)
%   weights  - N×1 vector (probabilities)
%   pct_cut  - cutoff as a decimal (e.g. 0.90 or 0.95)

% Normalize weights
weights = weights / max(sum(weights), eps);

% Sort
[vals_sorted, idx] = sort(values);
w_sorted = weights(idx);

% Cumulative distribution
cdf_vals = cumsum(w_sorted);

[~, p_idx] = min(abs(cdf_vals - pct_cut));

% Compute share
numerator = sum(vals_sorted(p_idx+1:end) .* w_sorted(p_idx+1:end));
denominator = sum(vals_sorted .* w_sorted);
share = numerator / max(denominator, eps);

end
