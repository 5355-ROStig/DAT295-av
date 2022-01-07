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

for(scenario in scenarios) {
  print("---------------------------------")
  print(paste("Prcessing scenario ", basename(scenario)))
  
  plot <- ggplot() + theme_minimal() + xlim(-4, 1) + ylim(0, 6.5)
  
  raw = list()
  smooth = list()
  minlist = list()
  lol = c()
  
  for(experiment in list.files(scenario, full.names = TRUE)) {
    output = tempfile(fileext = ".csv")
    print(paste("Running TTC analysis of ", experiment))
    system(paste("python3 ", ttcPath, " -f median -s 20 ", experiment, " ", output))
    data <- read.csv(output)
    data <- data[is.finite(rowSums(data)),] # Remove INF
    # Remove everything after 5s because we moved the cars manually in some experiments
    data <- subset(data, time<5)
    
    minrow = data[which.min(data$ttc),]
    lol <- c(lol, minrow[1,]$ttc)
    
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
  plot <- plot +
    stat_boxplot(geom ='errorbar', data = data.frame(lol), aes(x=0, y=lol), width=0.5) +
    geom_boxplot(lwd=0.75, data = data.frame(lol), aes(x=0, y=lol))
  #plot <- addList(plot, minlist)

  print(plot)
  ggsave(plot = plot,
         filename = paste(basename(scenario), ".pdf"),
         path = "datacap",
         device = "pdf")
}
