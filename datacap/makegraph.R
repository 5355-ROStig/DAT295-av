# Load dependencies...
depend <- function(name) {
  name <- as.character(substitute(name))
  if (!require(name, character.only = TRUE)) {
    print(paste(name, " missing, installing..."))
    install.packages(name)
  }
  library(name, character.only = TRUE)
}
depend("ggplot2")
depend("here")

# Set working directory to the Git root
setwd(here())

ttcPath = normalizePath(file.path("src", "ttc", "src", "gv_ttc_logger.py"))

scenarios <- list.dirs('datacap', recursive=FALSE)

# Variables used for making boxplots later
ttc = c()
ttc_scenario = c()

for(scenario in scenarios) {
  print("---------------------------------")
  print(paste("Prcessing scenario ", basename(scenario)))
  
  plot <- ggplot() + theme_minimal(base_size = 20) + xlim(-4, 1) + ylim(0, 6.5) +
    theme(plot.title = element_text(hjust = 0.5)) + ggtitle(basename(scenario))
  
  raw = list()
  smooth = list()
  minlist = list()

  for(experiment in list.files(scenario, full.names = TRUE)) {
    output = tempfile(fileext = ".csv")
    print(paste("Running TTC analysis of ", experiment))
    system(paste("python3 \"", ttcPath, "\" -f median -s 20 \"", experiment, "\" \"", output, "\"", sep = ''))
    data <- read.csv(output)
    data <- data[is.finite(rowSums(data)),] # Remove INF
    # Remove everything after 5s because cars were manually moved during some experiments
    data <- subset(data, time<10)

    minrow = data[which.min(data$ttc),]
    ttc <- c(ttc, minrow[1,]$ttc)
    ttc_scenario <- c(ttc_scenario, basename(scenario))
    
    data$time = data$time - minrow[1,]$time
    minrow$time = minrow$time - minrow[1,]$time

    # Add data to plot
    raw = c(raw, geom_line(data = data, aes(x = time, y = ttc), color="gray77"))
    smooth = c(smooth, geom_smooth(data = data,
                                  aes(x = time, y = ttc),
                                  se = FALSE,
                                  method = "loess",
                                  formula = y ~ x,
                                  color="gray40",
                                  linetype="dashed"))
    minlist = c(minlist, geom_point(data = minrow,
                                    aes(x = time, y = ttc),
                                    color = "black",
                                    size=2.5))
  }
  
  addList <- function(p, l){Reduce(function(x, y){x+y}, l, init=p)}
  plot <- addList(plot, raw)
  plot <- addList(plot, smooth)
  plot <- addList(plot, minlist)
  plot <- plot + labs(x = "Time to minimum time to collision (s)",
                      y = "Time to collision (s)")

  print(plot)
  ggsave(plot = plot,
         filename = paste(basename(scenario), ".pdf", sep = ''),
         path = "datacap",
         device = "pdf")
}

# Now, create final boxplots
frame <- data.frame(ttc = ttc, scenario = ttc_scenario)
plot <- ggplot(data = frame, aes(x=scenario, y=ttc)) +
        theme_minimal(base_size = 20) +
        theme(plot.title = element_text(hjust = 0.5)) +
        ylim(0, 3) + ggtitle("All scenarios") + 
        stat_boxplot(geom ='errorbar', width=0.5) +
        geom_boxplot() +
        labs(x = "Scenario", y = "Time to collision (s)") +
  geom_hline(aes(yintercept = median(ttc)), linetype = "dashed", size = 1)

print(plot)

ggsave(plot = plot,
       filename = "summary.pdf",
       path = "datacap",
       device = "pdf")

print(paste("Median for min TTC is", median(frame$ttc)))

######################################
# Create latency graph

file <- file.path("datacap", "latency.csv")
data = read.csv(file)

plot <- ggplot(data = data, aes(x = latency)) +
        theme_minimal(base_size = 20) +
        theme(plot.title = element_text(hjust = 0.5)) +
        xlim(0, 0.75) + ggtitle("System latency") +
        geom_histogram(binwidth = 0.025, origin = 0) +
        labs(x = "Samples", y = "Latency (s)") + geom_density() +
        geom_vline(aes(xintercept = median(latency)), linetype = "dashed", size = 1)

ggsave(plot = plot,
       filename = "latency.pdf",
       path = "datacap",
       device = "pdf")

print(paste("Median for latency is", median(data$latency)))
