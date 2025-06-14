/*
 * Copyright (C) 2008 Wim Taymans <wim.taymans at gmail.com>
 * Copyright (C) 2025 Vasily Evseenko <svpcom@p2ptech.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

static gboolean
timeout_cb (GstRTSPServer *server)
{
  GstRTSPSessionPool *pool = gst_rtsp_server_get_session_pool (server);
  gst_rtsp_session_pool_cleanup (pool);
  g_object_unref (pool);
  return TRUE;
}

static void
usage (const char *bin)
{
  fprintf (stderr,
      "Usage: %s  { h264 | h265 | opus }  [latency_ms]  [opus_clock]\n"
      "         h264/h265:  defaults latency to 200 ms\n"
      "         opus:       defaults latency 200 ms, clock-rate 48000\n",
      bin);
  exit (1);
}

int
main (int argc, char *argv[])
{
  /* ---------- command-line parsing ------------------------------------- */
  const char *media = NULL;
  gint   latency     = 200;      /* ms */
  gint   opus_rate   = 48000;    /* Hz */

  if (argc < 2)
    usage (argv[0]);

  media = argv[1];

  if (argc >= 3)
    latency = atoi (argv[2]);

  if (strcmp (media, "opus") == 0 && argc >= 4)
    opus_rate = atoi (argv[3]);

  if (strcmp (media, "h264") &&
      strcmp (media, "h265") &&
      strcmp (media, "opus"))
    usage (argv[0]);

  /* ---------- GStreamer / RTSP-server setup ---------------------------- */
  gst_init (&argc, &argv);

  GMainLoop         *loop    = g_main_loop_new (NULL, FALSE);
  GstRTSPServer     *server  = gst_rtsp_server_new ();
  GstRTSPMountPoints*mounts  = gst_rtsp_server_get_mount_points (server);
  GstRTSPMediaFactory*factory= gst_rtsp_media_factory_new ();
  char               launch[2048];

  if (!strcmp (media, "opus")) {
    /* Pure-audio (Opus) pipeline */
    snprintf (launch, sizeof (launch),
        "( udpsrc port=5600 "
        "! application/x-rtp,media=audio,payload=98,clock-rate=%d,encoding-name=OPUS "
        "! rtpjitterbuffer latency=%d "
        "! rtpopusdepay "
        "! rtpopuspay name=pay0 pt=98 )",
        opus_rate, latency);
  } else {
    /* Pure-video (H.264 or H.265) pipeline */
    int mode = (!strcmp (media, "h264")) ? 264 : 265;
    snprintf (launch, sizeof (launch),
        "( udpsrc port=5600 "
        "! application/x-rtp,media=video,clock-rate=90000,encoding-name=H%d "
        "! rtpjitterbuffer latency=%d "
        "! rtpH%ddepay "
        "! rtpH%dpay name=pay0 pt=96 )",
        mode, latency, mode, mode);
  }

  gst_rtsp_media_factory_set_launch (factory, launch);
  gst_rtsp_media_factory_set_shared (factory, TRUE);
  gst_rtsp_mount_points_add_factory (mounts, "/wfb", factory);
  g_object_unref (mounts);

  if (gst_rtsp_server_attach (server, NULL) == 0) {
    g_printerr ("failed to attach the server\n");
    return -1;
  }

  g_timeout_add_seconds (2, (GSourceFunc) timeout_cb, server);

  if (!strcmp (media, "opus"))
    g_print ("Opus stream (clock-rate %d Hz, latency %d ms) ready at rtsp://127.0.0.1:8554/wfb\n",
             opus_rate, latency);
  else
    g_print ("%s stream (latency %d ms) ready at rtsp://127.0.0.1:8554/wfb\n",
             media, latency);

  g_main_loop_run (loop);
  return 0;
}
