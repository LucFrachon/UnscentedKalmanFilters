library(ggplot2)

folder <- ""
radar_nis <- read.csv(paste0(folder, 'radar_nis.csv'), col.names = c('timestep', 'NIS'))
lidar_nis <- read.csv(paste0(folder, 'lidar_nis.csv'), col.names = c('timestep', 'NIS'))

first_timestep <- 1477010443050000
radar_nis$timestep <- (radar_nis$timestep - first_timestep) / 100000
lidar_nis$timestep <- (lidar_nis$timestep - first_timestep) / 100000

n_df_radar <- 3
n_df_lidar <- 2
thresh_radar <- qchisq(.95, df = 3)
thresh_lidar <- qchisq(.95, df = 2)

radar_nis$sensor <- "radar"
lidar_nis$sensor <- "lidar"

nis_data <- rbind(radar_nis, lidar_nis)

nis_data <- nis_data[order(nis_data$timestep),]

g <- ggplot(data = nis_data, aes(x = timestep, y = NIS)) +
     geom_line(aes(colour = sensor)) +
     geom_hline(yintercept = thresh_radar, colour = "turquoise") +
    geom_hline(yintercept = thresh_lidar, colour = "salmon")

exceeds_radar <- sum(nis_data[nis_data$sensor == "radar", "NIS"] > thresh_radar) / sum(nis_data$sensor == "radar")
exceeds_lidar <- sum(nis_data[nis_data$sensor == "lidar", "NIS"] > thresh_lidar) / sum(nis_data$sensor == "lidar")

print("Proportion of timesteps exceeding 95% threshold")
print(paste("RADAR:", exceeds_radar))
print(paste("LIDAR:", exceeds_lidar))

g