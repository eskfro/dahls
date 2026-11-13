function [k] = wp_selector(x, y, k_wp, WP, R)

wp_next = WP(:,k_wp+1);

d = sqrt( (x-wp_next(1))^2 + (y-wp_next(2))^2 );

if d < R
    k = k_wp+1;
else
    k = k_wp;
end