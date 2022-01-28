/*
 * dbmdx-snd-codec.c -- DBMDX ASoC platform driver
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "dbmdx-interface.h"
#ifdef DBMDX_VA_VE_SUPPORT
#include "dbmdx-usecase-config-def.h"
#endif

#define MAX_SUPPORTED_CHANNELS			2

struct snd_dbmdx_runtime_data {
	struct dbmdx_private *p;
	struct delayed_work pcm_start_capture_work;
	struct delayed_work pcm_stop_capture_work;
	struct workqueue_struct	*dbmdx_pcm_workq;
	unsigned int capture_in_progress;
	atomic_t command_in_progress;
	atomic_t number_of_cmds_in_progress;
};

struct snd_dbmdx_runtime_data g_runtime_data;

static DECLARE_WAIT_QUEUE_HEAD(dbmdx_wq);

int pcm_command_in_progress(struct snd_dbmdx_runtime_data *prtd,
	bool is_command_in_progress)
{
	if (is_command_in_progress) {
		if (!atomic_add_unless(&prtd->command_in_progress, 1, 1))
			return -EBUSY;
	} else {
		atomic_set(&prtd->command_in_progress, 0);
		atomic_dec(&prtd->number_of_cmds_in_progress);
		wake_up_interruptible(&dbmdx_wq);
	}

	return 0;
}

void wait_for_pcm_commands(struct snd_dbmdx_runtime_data *prtd)
{
	int ret;

	while (1) {
		wait_event_interruptible(dbmdx_wq,
			!(atomic_read(&prtd->command_in_progress)));

		ret = pcm_command_in_progress(prtd, 1);
		if (!ret)
			break;
	}
}

static int start_usecase_streaming(struct dbmdx_private *p)
{
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	int ret = 0;
	struct usecase_config *cur_usecase;

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	p->va_flags.buffering = 0;
	flush_work(&p->sv_work);

	cur_usecase = p->va_ve_flags.cur_usecase;

	if (cur_usecase && cur_usecase->usecase_supports_i2s_buffering) {
		p->lock(p);
		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
		if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_BUFFERING_WITH_HISTORY)
			ret = dbmdx_set_usecase_mode(p,
					(DBMDX_BUFFERING_WITH_BACKLOG_MASK |
					DBMDX_BUFFERING));
		else
			ret = dbmdx_set_usecase_mode(p, DBMDX_BUFFERING);
		p->unlock(p);
		if (ret) {
			dev_err(p->dev,
				"%s: Failed to set Usecase in Buffering mode\n",
				__func__);
			return -EIO;
		}

	} else if (p->pdata->default_streaming_usecase &&
			(p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_SET_DEFAULT_USECASE)) {
		ret = dbmdx_set_va_usecase_name(p,
				p->pdata->default_streaming_usecase);
		if (ret) {
			dev_err(p->dev,
				"%s: Failed to configure streaming usecase\n",
					__func__);
			return -EIO;
		}
		p->lock(p);
		dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
		if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_BUFFERING_WITH_HISTORY)
			ret = dbmdx_set_usecase_mode(p,
					(DBMDX_BUFFERING_WITH_BACKLOG_MASK |
					DBMDX_BUFFERING));
		else
			ret = dbmdx_set_usecase_mode(p, DBMDX_BUFFERING);

		p->unlock(p);
		if (ret) {
			dev_err(p->dev,
				"%s: Failed to set Buffering usecase mode\n",
				__func__);
			return -EIO;
		}

	} else {
		dev_err(p->dev,
		"%s: Usecase that supports streaming wasn't configured\n",
			__func__);
		return -EAGAIN;

	}
#endif
	return 0;
}

static int stop_usecase_streaming(struct dbmdx_private *p)
{
	int ret = 0;
#ifdef DBMDX_I2S_BUFFERING_SUPPORTED
	struct usecase_config *cur_usecase;

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	cur_usecase = p->va_ve_flags.cur_usecase;

	if (cur_usecase && p->va_ve_flags.usecase_mode == DBMDX_BUFFERING) {
		p->va_flags.buffering = 0;
		flush_work(&p->sv_work);
		if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_IDLE_AFTER_BUFFERING) {
			ret = dbmdx_set_va_usecase_name(p, "idle");
			if (ret) {
				dev_err(p->dev,
					"%s: Failed to set IDLE usecase\n",
					__func__);
				return -EIO;
			}
		} else {
			p->lock(p);
			dbmdx_set_power_mode(p, DBMDX_PM_ACTIVE);
			ret = dbmdx_set_usecase_mode(p, DBMDX_DETECTION);
			if (ret)
				dev_err(p->dev,
				"%s: Failed to set Usecase in Detection Mode\n",
					__func__);
			p->unlock(p);
		}
	}
#endif
	return ret;
}

static void  dbmdx_pcm_start_capture_work(struct work_struct *work)
{
	int ret;
	struct snd_dbmdx_runtime_data *prtd = container_of(
			work, struct snd_dbmdx_runtime_data,
			pcm_start_capture_work.work);
	struct dbmdx_private *p = prtd->p;

	pr_debug("%s:\n", __func__);

	wait_for_pcm_commands(prtd);

	if (prtd->capture_in_progress) {
		pr_debug("%s:Capture is already in progress\n", __func__);
		goto out;
	}

	prtd->capture_in_progress = 1;
	ret = start_usecase_streaming(p);
	if (ret < 0) {
		prtd->capture_in_progress = 0;
		pr_err("%s: failed to start capture device\n", __func__);
		goto out;
	}

out:
	pcm_command_in_progress(prtd, 0);
}

static void dbmdx_pcm_stop_capture_work(struct work_struct *work)
{
	int ret;
	struct snd_dbmdx_runtime_data *prtd = container_of(
			work, struct snd_dbmdx_runtime_data,
			pcm_stop_capture_work.work);
	struct dbmdx_private *p = prtd->p;

	pr_debug("%s:\n", __func__);

	wait_for_pcm_commands(prtd);

	if (!(prtd->capture_in_progress)) {
		pr_debug("%s:Capture is not in progress\n", __func__);
		goto out;
	}

	ret = stop_usecase_streaming(p);

	if (ret < 0)
		pr_err("%s: failed to stop pcm streaming\n", __func__);

	prtd->capture_in_progress = 0;
out:
	pcm_command_in_progress(prtd, 0);
}


static int dbmdx_pcm_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct dbmdx_private *p = snd_soc_codec_get_drvdata(rtd->codec);
	struct snd_dbmdx_runtime_data *prtd = &g_runtime_data;
	int ret = 0;
	int num_of_active_cmds;

	if (!p || !p->device_ready) {
		pr_err("%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	if (p->pdata->alsa_streaming_options &
			DBMDX_ALSA_STREAMING_DISABLED)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		num_of_active_cmds =
				atomic_read(&prtd->number_of_cmds_in_progress);
		dev_dbg(p->dev,	"%s: Number of active commands=%d\n", __func__,
			num_of_active_cmds);

		atomic_inc(&prtd->number_of_cmds_in_progress);
		ret = queue_delayed_work(prtd->dbmdx_pcm_workq,
					&prtd->pcm_start_capture_work,
				msecs_to_jiffies(num_of_active_cmds*100));
		if (!ret) {
			dev_warn(p->dev,
				"%s: Start command is already pending\n",
				__func__);
			atomic_dec(&prtd->number_of_cmds_in_progress);
		} else {
			dev_dbg(p->dev,	"%s: Start has been scheduled\n",
				__func__);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		num_of_active_cmds =
				atomic_read(&prtd->number_of_cmds_in_progress);
		dev_dbg(p->dev,	"%s: Number of active commands=%d\n", __func__,
			num_of_active_cmds);
		atomic_inc(&prtd->number_of_cmds_in_progress);
		ret = queue_delayed_work(prtd->dbmdx_pcm_workq,
					&prtd->pcm_stop_capture_work,
				msecs_to_jiffies(num_of_active_cmds*100));
		if (!ret) {
			dev_dbg(p->dev,	"%s: Stop command is already pending\n",
				__func__);
			atomic_dec(&prtd->number_of_cmds_in_progress);
		} else {
			dev_dbg(p->dev,	"%s: Stop has been scheduled\n",
				__func__);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static struct snd_soc_codec_driver soc_codec_dev_dbmdx = {
	.probe   = dbmdx_dev_probe,
	.remove  = dbmdx_dev_remove,
};

static struct snd_soc_dai_ops dbmdx_dai_ops = {
	.trigger		= dbmdx_pcm_trigger,
};


/* DBMDX codec DAI: */
static struct snd_soc_dai_driver dbmdx_codec_dai = {
		.name = "DBMDX_i2s_codec",
		.playback = {
			.channels_min	= 2,
			.channels_max	= MAX_SUPPORTED_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.channels_min	= 2,
			.channels_max	= MAX_SUPPORTED_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &dbmdx_dai_ops,
};

int dbmdx_snd_init(struct dbmdx_private *p)
{
	struct snd_dbmdx_runtime_data *prtd = &g_runtime_data;
	/* DBMDX snd codec registration */
	prtd->p = p;

	atomic_set(&prtd->command_in_progress, 0);
	atomic_set(&prtd->number_of_cmds_in_progress, 0);

	INIT_DELAYED_WORK(&prtd->pcm_start_capture_work,
		dbmdx_pcm_start_capture_work);
	INIT_DELAYED_WORK(&prtd->pcm_stop_capture_work,
		dbmdx_pcm_stop_capture_work);
	prtd->dbmdx_pcm_workq = create_workqueue("dbmdx-pcm-wq");
	if (!prtd->dbmdx_pcm_workq) {
		pr_err("%s: Could not create pcm workqueue\n", __func__);
		return -EIO;
	}

	return snd_soc_register_codec(p->dev, &soc_codec_dev_dbmdx,
			&dbmdx_codec_dai, 1);
}

void dbmdx_snd_deinit(struct dbmdx_private *p)
{
	struct snd_dbmdx_runtime_data *prtd = &g_runtime_data;

	flush_delayed_work(&prtd->pcm_start_capture_work);
	flush_delayed_work(&prtd->pcm_stop_capture_work);
	queue_delayed_work(prtd->dbmdx_pcm_workq,
		&prtd->pcm_stop_capture_work,
		msecs_to_jiffies(0));
	flush_delayed_work(&prtd->pcm_stop_capture_work);
	flush_workqueue(prtd->dbmdx_pcm_workq);
	usleep_range(10000, 11000);
	destroy_workqueue(prtd->dbmdx_pcm_workq);

	/*DBMDX snd codec unregister */
	snd_soc_unregister_codec(p->dev);
}
