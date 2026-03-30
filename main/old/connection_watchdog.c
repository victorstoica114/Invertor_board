#include <stdio.h>
#include <time.h>
#include <stdbool.h>

#define TIMEOUT_THRESHOLD 5 // seconds
#define ALERT_MESSAGE "Data source has timed out!"

time_t last_update_time;

void update_time() {
    last_update_time = time(NULL);
}

bool is_data_fresh() {
    return (difftime(time(NULL), last_update_time) < TIMEOUT_THRESHOLD);
}

void check_data_freshness() {
    if (!is_data_fresh()) {
        trigger_alert();
    }
}

void trigger_alert() {
    printf("%s\n", ALERT_MESSAGE);
}

int main() {
    // Initialize last update time
    update_time();
    
    // Simulate monitoring process
    while (true) {
        check_data_freshness();
        // Simulate data update for demonstration
        sleep(1); // for a real application, this would wait for new data to arrive
        update_time();  // update time on data update
    }
    return 0;
}